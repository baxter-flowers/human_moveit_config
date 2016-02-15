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
        self.groups = {}
        self.groups['head'] = moveit_commander.MoveGroupCommander('head')
        self.groups['right_arm'] = moveit_commander.MoveGroupCommander('right_arm')
        self.groups['left_arm'] = moveit_commander.MoveGroupCommander('left_arm')
        # initialize common links per group
        self.group_links = {}
        self.group_links['head'] = ['torso', 'head_tip']
        self.group_links['right_arm'] = ['right_upper_arm', 'right_forearm', 'right_hand_tip']
        self.group_links['left_arm'] = ['left_upper_arm', 'left_forearm', 'left_hand_tip']
        # initialize end-effectors dict
        self.end_effectors = {}
        # fill both dict
        for key, value in self.groups.iteritems():
            self.end_effectors[key] = value.get_end_effector_link()
        # fill the disct of active joints by links
        self.joint_by_links = {}
        self.joint_by_links['torso'] = ['spine_0', 'spine_1', 'spine_2']
        self.joint_by_links['head_tip'] = ['neck_0', 'neck_1', 'neck_2']
        sides = ['right', 'left']
        for s in sides:
            self.joint_by_links[s+'_upper_arm'] = [s+'_shoulder_0', s+'_shoulder_1', s+'_shoulder_2']
            self.joint_by_links[s+'_forearm'] = [s+'_elbow_0', s+'_elbow_1']
            self.joint_by_links[s+'_hand_tip'] = [s+'_wrist_0', s+'_wrist_1']

    def forward_kinematic(self, group_name, joint_values, links=None, joint_names=None):
        def compute_fk_client(joints):
            rospy.wait_for_service('compute_fk')
            try:
                compute_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
                header = Header()
                header.stamp = rospy.Time.now()
                header.frame_id = '/hip'

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

    def inverse_kinematic(self, group_name, desired_poses, tolerance=0.1, fill=False, links=None):
        def compute_ik_client():
            rospy.wait_for_service('compute_ik')
            try:
                compute_ik = rospy.ServiceProxy('compute_ik', GetHumanIK)
                # convert the desired poses to geometry_msgs
                poses = []
                for pose in desired_poses:
                    poses.append(transformations.list_to_pose(pose).pose)
                res = compute_ik(group_name, links, poses, active_joints, tolerance)
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

    def get_joint_values(self, group_name):
        return self.groups[group_name].get_current_joint_values()

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

    def move_group_by_joints(self, group_name, joint_values, execute=True):
        self.groups[group_name].set_joint_value_target(joint_values)
        if execute:
            # get current joints
            self.groups[group_name].go(wait=True)
            current = self.groups[group_name].get_current_joint_values()
            dist = np.linalg.norm(np.array(current)-np.array(joint_values))
            return dist < 0.001
        return True

    def joint_limits_by_group(self, group_name, joint_names=None):
        if joint_names is None or not joint_names:
            joint_names = self.groups[group_name].get_active_joints()
        xml_urdf = rospy.get_param('robot_description')
        dict_urdf = xmltodict.parse(xml_urdf)
        joints_urdf = []
        joints_urdf.append([j['@name'] for j in dict_urdf['robot']['joint'] if j['@name'] in joint_names])
        joints_urdf.append([[float(j['limit']['@lower']), float(j['limit']['@upper'])]
                           for j in dict_urdf['robot']['joint'] if j['@name'] in joint_names])
        # reorder the joints limits
        limits = {}
        limits['joint_names'] = joint_names
        limits['limits'] = [joints_urdf[1][joints_urdf[0].index(name)] for name in joint_names]
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

    def jacobian(self, group_name, joint_values, link=None, ref_point=None):
        def compute_jacobian_srv():
            rospy.wait_for_service('compute_jacobian')
            try:
                compute_jac = rospy.ServiceProxy('compute_jacobian', GetJacobian)
                js = JointState()
                js.position = joint_values
                p = Point(x=ref_point[0], y=ref_point[1], z=ref_point[2])
                # call the service
                res = compute_jac(group_name, link, js, p)
                # reorganize the jacobian
                jac_array = np.array(res.jacobian).reshape((6, res.nb_joints))
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
