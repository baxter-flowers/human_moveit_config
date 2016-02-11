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


class HumanModel:
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
        # initialize joint limits dict
        self.joint_limits = {}
        # fill both dict
        for key, value in self.groups.iteritems():
            self.end_effectors[key] = value.get_end_effector_link()
            self.joint_limits[key] = self.joint_limits_by_group(key)['limits']

    def forward_kinematic(self, group_name, joint_values, links=None):
        def compute_fk_client(group):
            rospy.wait_for_service('compute_fk')
            try:
                compute_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
                header = Header()
                header.stamp = rospy.Time.now()
                header.frame_id = '/hip'

                rs = RobotState()
                rs.joint_state.header = header
                rs.joint_state.name = group.get_active_joints()
                rs.joint_state.position = joint_values

                res = compute_fk(header, links, rs)
                return res.pose_stamped
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e

        if links is None:
            links = [self.end_effectors[group_name]]
        pose_stamped_list = compute_fk_client(self.groups[group_name])
        # transform it in a list of pose
        if len(pose_stamped_list) == 1:
            return transformations.pose_to_list(pose_stamped_list[0].pose)
        pose_list = []
        for pose_stamped in pose_stamped_list:
            pose_list.append(transformations.pose_to_list(pose_stamped.pose))
        return pose_list

    def get_joint_values(self, group_name):
        return self.groups[group_name].get_current_joint_values()

    def move_group_by_joints(self, group_name, joint_values, execute=True):
        self.groups[group_name].set_joint_value_target(joint_values)
        if execute:
            # get current joints
            self.groups[group_name].go(wait=True)
            current = self.groups[group_name].get_current_joint_values()
            dist = np.linalg.norm(np.array(current)-np.array(joint_values))
            return dist < 0.001
        return True

    def joint_limits_by_group(self, group_name):
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

    def get_random_joint_values(self, group_name):
        return self.groups[group_name].get_random_joint_values()

    def jacobian(self, group_name, joint_values, link=None, ref_point=None):
        def compute_jacobian_srv():
            print "try to call jacobian server"
            rospy.wait_for_service('compute_jacobian')
            print "jacobian server online"
            try:
                compute_jac = rospy.ServiceProxy('compute_jacobian', GetJacobian)
                js = JointState()
                js.position = joint_values
                # call the service
                res = compute_jac(group_name, link, js, ref_point)
                # reorganize the jacobian
                jac_array = np.array(res.jacobian).reshape((6, res.nb_joints))
                return jac_array
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e

        # assign values
        if link is None:
            link = self.end_effectors[group_name]
        if ref_point is None:
            ref_point = Point(x=0, y=0, z=0)
        # return the jacobian
        return compute_jacobian_srv()
