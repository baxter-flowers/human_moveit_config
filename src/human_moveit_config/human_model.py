#!/usr/bin/env python
import rospy
import xmltodict
import moveit_commander
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import transformations
import numpy as np
from human_moveit_config.srv import GetJacobian
from human_moveit_config.srv import GetHumanIK


class HumanModel(object):
    def __init__(self):
        self.robot_commander = moveit_commander.RobotCommander()
        self.joint_publisher = rospy.Publisher('/human/set_joint_values', JointState, queue_size=1)
        self.groups = {}
        self.groups['head'] = moveit_commander.MoveGroupCommander('Head')
        self.groups['right_arm'] = moveit_commander.MoveGroupCommander('RightArm')
        self.groups['left_arm'] = moveit_commander.MoveGroupCommander('LeftArm')
        self.groups['right_leg'] = moveit_commander.MoveGroupCommander('RightLeg')
        self.groups['left_leg'] = moveit_commander.MoveGroupCommander('LeftLeg')
        self.groups['upper_body'] = moveit_commander.MoveGroupCommander('UpperBody')
        self.groups['lower_body'] = moveit_commander.MoveGroupCommander('LowerBody')
        self.groups['whole_body'] = moveit_commander.MoveGroupCommander('WholeBody')
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
        # initialize common links per group
        self.group_links = {}
        self.group_links['head'] = ['torso', 'head_tip']
        # fill the disct of active joints by links
        self.joint_by_links = {}
        self.joint_by_links['torso'] = ['spine_0', 'spine_1', 'spine_2']
        self.joint_by_links['head_tip'] = ['neck_0', 'neck_1', 'neck_2']
        sides = ['right', 'left']
        for s in sides:
            self.group_links[s+'_arm'] = [s+'_upper_arm', s+'_forearm', s+'_hand_tip']
            self.group_links[s+'_leg'] = [s+'_thigh', s+'_shin', s+'_foot_tip']
            # arm links
            self.joint_by_links[s+'_upper_arm'] = [s+'_shoulder_0', s+'_shoulder_1', s+'_shoulder_2']
            self.joint_by_links[s+'_forearm'] = [s+'_elbow_0', s+'_elbow_1']
            self.joint_by_links[s+'_hand_tip'] = [s+'_wrist_0', s+'_wrist_1']
            # leg links
            self.joint_by_links[s+'_thigh'] = [s+'_hip_0', s+'_hip_1', s+'_hip_2']
            self.joint_by_links[s+'_shin'] = [s+'_knee']
            self.joint_by_links[s+'_foot_tip'] = [s+'_ankle_0', s+'_ankle_1']

    def extract_group_joints(self, group_name, joint_state):
        active = self.groups[group_name].get_active_joints()
        res = []
        for joint in active:
            index = joint_state.name.index(joint)
            res.append(joint_state.position[index])
        return res

    def full_forward_kinematic(self, joint_state, body='whole_body'):
        # initialize end-effector dict
        end_effectors_pose = {}
        # calculate the forward kinematic by group
        body_joints = self.extract_group_joints(body, joint_state)
        fk = self.forward_kinematic(body, body_joints, links=self.end_effectors[body])
        # add the result in a dictionnary
        for i in range(len(fk)):
            end_effectors_pose[self.end_effectors[body][i]] = fk[i]
        return end_effectors_pose

    def forward_kinematic(self, group_name, joint_values, base='/hip', links=None, joint_names=None):
        def compute_fk_client(joints):
            rospy.wait_for_service('compute_fk')
            try:
                compute_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
                header = Header()
                header.stamp = rospy.Time.now()
                header.frame_id = base
                rs = RobotState()
                rs.joint_state.header = header
                rs.joint_state.name = group.get_active_joints()
                rs.joint_state.position = joints
                res = compute_fk(header, links, rs)
                return res.pose_stamped
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e

        group = self.groups[group_name]
        # if joint_names is None the joints vector is supposed to be full
        if joint_names is None or not joint_names:
            assert len(group.get_active_joints()) == len(joint_values)
            joints = joint_values
        else:
            # get the full vector joints
            joints = self.set_joint_values(group_name, joint_values, joint_names)
        # if link is None assume it is the end-effector
        if links is None:
            links = [self.end_effectors[group_name]]
        if type(links) is not list:
            links = [links]
        pose_stamped_list = compute_fk_client(joints)
        # transform it in a list of pose
        if len(pose_stamped_list) == 1:
            return transformations.pose_to_list(pose_stamped_list[0].pose)
        pose_list = []
        for pose_stamped in pose_stamped_list:
            pose_list.append(transformations.pose_to_list(pose_stamped.pose))
        return pose_list

    def inverse_kinematic(self, group_name, desired_poses, tolerance=0.1, continuity=False, fill=False, links=None):
        def compute_ik_client():
            rospy.wait_for_service('compute_human_ik')
            try:
                compute_ik = rospy.ServiceProxy('compute_human_ik', GetHumanIK)
                # convert the desired poses to geometry_msgs
                poses = []
                for pose in desired_poses:
                    poses.append(transformations.list_to_pose(pose).pose)
                res = compute_ik(group_name, links, poses, active_joints, tolerance, continuity)
                return list(res.joint_state.position)
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e
        if links is None:
            links = [self.end_effectors[group_name]]
            active_joints = []
        else:
            if type(links) is not list:
                links = [links]
            # get the list of active joints
            active_joints = self.get_joint_by_links(group_name, links, fill)
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

    def set_joint_values(self, group_name, joint_values, joint_names):
        current_values = self.get_joint_values(group_name)
        # get names of active joints
        active_joints = self.groups[group_name].get_active_joints()
        for i in range(len(joint_names)):
            index = active_joints.index(joint_names[i])
            # replace the current value with the desired value
            current_values[index] = joint_values[i]
        # return the modified vector
        return current_values

    def get_joint_index(self, group_name, joint_names):
        indexes = []
        active_joints = self.groups[group_name].get_active_joints()
        for name in joint_names:
            indexes.append(active_joints.index(name))
        return indexes

    def get_current_state(self):
        return self.robot_commander.get_current_state()

    def send_state(self, joint_state, wait=True):
        self.joint_publisher.publish(joint_state)
        if wait:
            rospy.sleep(0.1)

    def send_joint_values(self, joint_names, joint_values, wait=True):
        # get the current state
        rs = self.robot_commander.get_current_state()
        js = rs.joint_state
        # replace the joint values in the joint state
        position = list(js.position)
        for i in range(len(joint_values)):
            index = js.name.index(joint_names[i])
            position[index] = joint_values[i]
        js.position = position
        self.send_state(js, wait)

    def get_joint_by_links(self, group_name, links, fill=True):
        joints = []
        if type(links) is not list:
            links = [links]
        for i in range(len(links)):
            supposed_index = self.group_links[group_name].index(links[i])
            if supposed_index == i or not fill:
                joints += self.joint_by_links[links[i]]
            else:
                for j in range(i, supposed_index+1):
                    link = self.group_links[group_name][j]
                    joints += self.joint_by_links[link]
        # remove dupblicates
        seen = set()
        seen_add = seen.add
        return [x for x in joints if not (x in seen or seen_add(x))]

    def move_group_by_joints(self, group_name, joint_values):
        joint_names = self.groups[group_name].get_active_joints()
        self.send_joint_values(joint_names, joint_values, wait=True)

    def get_joint_limits(self, joint_names=None):
        if joint_names is None:
            rs = self.get_current_state()
            joint_names = rs.joint_state.name
        xml_urdf = rospy.get_param('robot_description')
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

    def get_random_joint_values(self, group_name, joint_names=None):
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
            rs = self.get_current_state()
            return rs.joint_state.name
        else:
            return self.groups[group_name].get_active_joints()

    def get_random_state(self):
        rs = self.robot_commander.get_current_state()
        positions = list(rs.joint_state.position)
        for group_name, group in self.groups.iteritems():
            joints = group.get_random_joint_values()
            joint_names = group.get_active_joints()
            for i in range(len(joints)):
                index = rs.joint_state.name.index(joint_names[i])
                positions[index] = joints[i]
        rs.joint_state.position = positions
        return rs.joint_state

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
                return jac_array
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e

        # assign values
        if link is None:
            link = self.end_effectors[group_name]
        if ref_point is None:
            ref_point = [0, 0, 0]
        # return the jacobian
        return compute_jacobian_srv()
