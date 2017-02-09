#!/usr/bin/env python
import rospy
import xmltodict
from moveit_commander import MoveGroupCommander
from moveit_commander import RobotCommander
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import transformations
import numpy as np
from human_moveit_config.srv import GetJacobian
from human_moveit_config.srv import GetHumanIK
from copy import deepcopy
import re
from human_moveit_config.urdf_reader import URDFReader


class HumanModel(object):
    def __init__(self, description='human_description', prefix='human', control=False):
        self.description = description
        self.robot_commander = RobotCommander(description)
        if control:
            self.joint_publisher = rospy.Publisher('/human/set_joint_values', JointState, queue_size=1)
        self.groups = {}
        self.groups['head'] = MoveGroupCommander('Head', description)
        self.groups['right_arm'] = MoveGroupCommander('RightArm', description)
        self.groups['left_arm'] = MoveGroupCommander('LeftArm', description)
        self.groups['right_leg'] = MoveGroupCommander('RightLeg', description)
        self.groups['left_leg'] = MoveGroupCommander('LeftLeg', description)
        self.groups['upper_body'] = MoveGroupCommander('UpperBody', description)
        self.groups['lower_body'] = MoveGroupCommander('LowerBody', description)
        self.groups['whole_body'] = MoveGroupCommander('WholeBody', description)
        # initialize end-effectors dict
        self.end_effectors = {}
        # fill both dict
        for key, value in self.groups.iteritems():
            self.end_effectors[key] = value.get_end_effector_link()
        # add the list of end-effectors for bodies
        self.end_effectors['upper_body'] = [self.end_effectors['head'],
                                            self.end_effectors['right_arm'],
                                            self.end_effectors['left_arm']]
        self.end_effectors['lower_body'] = [self.end_effectors['right_leg'],
                                            self.end_effectors['left_leg']]
        self.end_effectors['whole_body'] = self.end_effectors['upper_body'] + self.end_effectors['lower_body']
        self.prefix = prefix
        self.urdf_reader = URDFReader()

        rospy.wait_for_service('compute_fk')
        self.compute_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
        self.current_state = self.get_initial_state()

    def find_common_root(self, link1, link2):
        return self.urdf_reader.find_common_root(link1, link2)

    def get_links_chain(self, from_link, to_link):
        return self.urdf_reader.get_links_chain(from_link, to_link)

    def get_joints_chain(self, from_link, to_link):
        return self.urdf_reader.get_joints_chain(from_link, to_link)

    def extract_group_joints(self, group_name, joint_state):
        active = self.groups[group_name].get_active_joints()
        res = []
        for joint in active:
            index = joint_state.name.index(joint)
            res.append(joint_state.position[index])
        return res

    def forward_kinematic(self, joint_state, base='base', links=None):
        def compute_fk_client():
            try:
                header = Header()
                header.stamp = rospy.Time.now()
                header.frame_id = self.prefix + '/base'
                rs = RobotState()
                rs.joint_state = joint_state
                res = self.compute_fk(header, links, rs)
                return res.pose_stamped
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e
                # in case of troubles return 0 pose stamped
                return []

        if links is None:
            links = self.end_effectors['whole_body']
        if type(links) is not list:
            if links == "all":
                links = self.get_link_names('whole_body')
            else:
                links = [links]
        # check that the base is in links
        if base != 'base' and base not in links:
            links.append(base)
        pose_stamped_list = compute_fk_client()
        if not pose_stamped_list:
            return {}
        # transform it in a dict of poses
        pose_dict = {}
        if base != 'base':
            tr_base = transformations.pose_to_list(pose_stamped_list[links.index(base)].pose)
            inv_base = transformations.inverse_transform(tr_base)
            for i in range(len(links)):
                if links[i] != base:
                    tr = transformations.pose_to_list(pose_stamped_list[i].pose)
                    pose_dict[links[i]] = transformations.multiply_transform(inv_base, tr)
        else:
            for i in range(len(links)):
                pose_dict[links[i]] = transformations.pose_to_list(pose_stamped_list[i].pose)
        return pose_dict

    def inverse_kinematic(self, desired_poses, fixed_joints={}, tolerance=0.001, group_names='whole_body', seed=None):
        def compute_ik_client():
            rospy.wait_for_service('compute_human_ik')
            try:
                compute_ik = rospy.ServiceProxy('compute_human_ik', GetHumanIK)
                res = compute_ik(poses, fixed_joint_state, tolerance, group_names, seed)
                return res.joint_state
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e
                # in case of failure return T pose
                return self.get_initial_state()
        if seed is None:
            seed = self.get_current_state()
        if group_names is not list:
            group_names = [group_names]
        # convert the desired poses to PoseStamped
        poses = []
        for key, value in desired_poses.iteritems():
            pose = transformations.list_to_pose(value)
            pose.header.frame_id = key
            poses.append(pose)
        # convert the fixed joints to joint state
        fixed_joint_state = JointState()
        for key, value in fixed_joints.iteritems():
            fixed_joint_state.name += [key]
            fixed_joint_state.position += [value]
        return compute_ik_client()

    def get_joint_values(self, group_name, joint_names=None):
        if joint_names is None or not joint_names:
            return self.groups[group_name].get_current_joint_values()
        else:
            joint_values = self.groups[group_name].get_current_joint_values()
            res = []
            # get names of active joints
            active_joints = self.groups[group_name].get_active_joints()
            for i in range(len(joint_names)):
                index = active_joints.index(joint_names[i])
                # get current values of desired joints
                res.append(joint_values[index])
            return res

    def get_current_state(self):
        return deepcopy(self.current_state)

    def send_state(self, state, wait=False):
        if isinstance(state, RobotState):
            state = state.joint_state
        self.joint_publisher.publish(state)
        for i, joint in enumerate(state.name):
            self.current_state.position[self.current_state.name.index(joint)] = state.position[i]

    def send_joint_values(self, joint_names, joint_values):
        # get the current state
        rs = self.robot_commander.get_current_state()
        js = rs.joint_state
        # replace the joint values in the joint state
        position = list(js.position)
        for i in range(len(joint_values)):
            index = js.name.index(joint_names[i])
            position[index] = joint_values[i]
        js.position = position
        self.send_state(js)

    def move_group_by_joints(self, group_name, joint_values):
        joint_names = self.groups[group_name].get_active_joints()
        self.send_joint_values(joint_names, joint_values, wait=True)

    def get_joint_limits(self, joint_names=None):
        if joint_names is None:
            js = self.get_current_state()
            joint_names = js.name
        xml_urdf = rospy.get_param(self.description)
        dict_urdf = xmltodict.parse(xml_urdf)
        joints_urdf = []
        joints_urdf.append([j['@name'] for j in dict_urdf['robot']['joint'] if j['@name'] in joint_names])
        joints_urdf.append([[float(j['limit']['@lower']), float(j['limit']['@upper'])]
                           for j in dict_urdf['robot']['joint'] if j['@name'] in joint_names])
        # reorder the joints limits
        limits = [joints_urdf[1][joints_urdf[0].index(name)] for name in joint_names]
        return limits

    def joint_limits_by_group(self, group_name):
        joint_names = self.groups[group_name].get_active_joints()
        joint_limits = self.get_joint_limits(joint_names)
        limits = {}
        limits['joint_names'] = joint_names
        limits['limits'] = joint_limits
        return limits

    def get_random_joint_values(self, group_name='whole_body', joint_names=None):
        # if all joints can move
        if joint_names is None or not joint_names:
            return self.groups[group_name].get_random_joint_values()
        else:
            random_joints = self.groups[group_name].get_random_joint_values()
            res = []
            # get names of active joints
            active_joints = self.groups[group_name].get_active_joints()
            for i in range(len(joint_names)):
                index = active_joints.index(joint_names[i])
                # replace the current value with the random value
                res.append(random_joints[index])
            return res

    def get_joint_names(self, group_name=None):
        if group_name is None:
            js = self.get_current_state()
            return js.name
        else:
            return self.groups[group_name].get_active_joints()

    def get_link_names(self, group_name='whole_body'):
        robot_group_name = self.groups[group_name].get_name()
        return self.robot_commander.get_link_names(robot_group_name)

    def get_random_state(self):
        js = self.get_current_state()
        positions = list(js.position)
        for group_name, group in self.groups.iteritems():
            joints = group.get_random_joint_values()
            joint_names = group.get_active_joints()
            for i in range(len(joints)):
                index = js.name.index(joint_names[i])
                positions[index] = joints[i]
        js.position = positions
        return js

    def get_initial_state(self):
        js = self.robot_commander.get_current_state().joint_state
        # put the model in T pose, i.e all joints values at 0
        js.position = np.zeros(len(js.position))
        return js

    def jacobian(self, group_name, joint_state, use_quaternion=False, link=None, ref_point=None):
        def compute_jacobian_srv():
            rospy.wait_for_service('compute_jacobian')
            try:
                compute_jac = rospy.ServiceProxy('compute_jacobian', GetJacobian)
                js = JointState()
                js.name = self.get_joint_names(group_name)
                js.position = self.extract_group_joints(group_name, joint_state)
                p = Point(x=ref_point[0], y=ref_point[1], z=ref_point[2])
                # call the service
                res = compute_jac(group_name, link, js, p, use_quaternion)
                # reorganize the jacobian
                jac_array = np.array(res.jacobian).reshape((res.nb_rows, res.nb_cols))
                # reorder the jacobian wrt to the joint state
                ordered_jac = np.zeros((len(jac_array), len(joint_state.name)))
                for i, n in enumerate(js.name):
                    ordered_jac[:, joint_state.name.index(n)] = jac_array[:, i]
                return ordered_jac
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e
        #  compute the jacobian only for chains
        # if group_name not in ['right_arm', 'left_arm', 'head', 'right_leg', 'left_leg']:
        #     rospy.logerr('The Jacobian matrix can only be computed on kinematic chains')
        #     return []
        # assign values
        if link is None:
            link = self.end_effectors[group_name]
        if ref_point is None:
            ref_point = [0, 0, 0]
        # return the jacobian
        return compute_jacobian_srv()

    def nearest_neighbour(self, fk_dict, scale_orient=40):
        def query_database(link, fk):
            x_vector = deepcopy(fk[0])
            if self.with_orient:
                x_vector += (np.array(fk[1]) / scale_orient).tolist()
            res = self.trees[link]['tree'].query(x_vector)
            joints = self.trees[link]['data']['data'][res[1]][:-7]
            joint_names = self.trees[link]['data']['names'][:-7]
            return joints.tolist(), joint_names.tolist()

        state = JointState()
        for key, value in fk_dict.iteritems():
            joints, joint_names = query_database(key, value)
            state.position += joints
            state.name += joint_names
        return state

    def get_segment_poses(self, joint_state, groups):
        regex = re.compile('_\d')
        segments = []
        fk_dict = self.forward_kinematic(joint_state, links='all')
        for g in groups:
            chain = self.get_link_names(g)
            filtered = [i for i in chain if not regex.search(i)]
            seg_dict = {}
            seg_dict['links'] = filtered
            seg_dict['poses'] = [fk_dict[i] for i in filtered]
            segments.append(seg_dict)
        return segments
