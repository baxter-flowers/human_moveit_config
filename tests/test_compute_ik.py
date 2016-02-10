#!/usr/bin/env python
import unittest
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from std_msgs.msg import Header
import transformations
import numpy as np
import math
from human_moveit_config.optimize_ik import Optimizer
from human_moveit_config.human_model import HumanModel


def compute_fk_client(group, joint_values, links):
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


class TestTransformation(unittest.TestCase):
    def test_compute_ik(self):
        rospy.init_node('test_compute_ik')
        human = HumanModel()
        opti = Optimizer()

        start_head_joints = np.zeros(6).tolist()
        sides = ['right', 'left']
        start_arm_joints = np.zeros((2, 7)).tolist()
        # move back to initial pose
        human.move_group_by_joints('head', start_head_joints)
        for j in range(2):
            human.move_group_by_joints(sides[j]+'_arm', start_arm_joints[j])

        start = time.time()
        for i in range(10):
            # get random pose
            head_joint_values = human.get_random_joint_values('head')

            HHP_T_OHP = [[0.0, 0.0, 0.0], [0, 0, 0, 1]]
            OH_T_HH = [[0.0, 0.0, 0.0], [0, 0, 0, 1]]

            arm_joint_values = []
            for j in range(2):
                arm_joint_values.append(human.get_random_joint_values(sides[j]+'_arm'))
            # get fk
            head_des_pose = HHP_T_OHP
            # here we should replace by observed transfromation from optitrack
            head_des_pose = transformations.multiply_transform(head_des_pose, human.forward_kinematic('head', head_joint_values))
            head_des_pose = transformations.multiply_transform(head_des_pose, OH_T_HH)

            # create the dict of desired poses
            des_poses = {}
            des_poses['head_tip'] = head_des_pose

            # move the torso
            human.move_group_by_joints('head', head_joint_values)

            # arm_des_pose = []
            # arm_des_pose.append(human.forward_kinematic(sides[0]+'_arm', arm_joint_values[0]))
            # arm_des_pose.append(human.forward_kinematic(sides[1]+'_arm', arm_joint_values[1]))
            # # move the arms
            # for j in range(2):
            #     human.move_group_by_joints(sides[j]+'_arm', arm_joint_values[j])

            # move back to initial pose
            human.move_group_by_joints('head', start_head_joints)

            # for j in range(2):
            #     human.move_group_by_joints(sides[j]+'_arm', start_arm_joints[j])

            # compute the ik
            opti.compute_ik(des_poses)

            # get the joint values
            current_head_joints = human.get_joint_values('head')
            head_ik = human.forward_kinematic('head', current_head_joints)

            # current_arm_joints = []
            # arm_ik = []
            # for j in range(2):
            #     current_arm_joints.append(human.get_joint_values(sides[j]+'_arm'))
            #     arm_ik.append(human.forward_kinematic(sides[j]+'_arm', current_arm_joints[j]))
            # assert equality between initial joint values and ik
            np.testing.assert_almost_equal(head_ik[0], head_des_pose[0], decimal=3)
            np.testing.assert_almost_equal(np.absolute(head_ik[1]), np.absolute(head_des_pose[1]), decimal=3)

            # np.testing.assert_almost_equal(current_head_joints, head_joint_values, decimal=3)
            # np.testing.assert_almost_equal(current_arm_joints[0], arm_joint_values[0], decimal=3)
            # np.testing.assert_almost_equal(current_arm_joints[1], arm_joint_values[1], decimal=3)

            # move back to initial pose
            human.move_group_by_joints('head', start_head_joints)
            # for j in range(2):
            #     human.move_group_by_joints(sides[j]+'_arm', start_arm_joints[j])

            print 'run number '+str(i)+' passed'
        print time.time() - start

if __name__ == '__main__':
    unittest.main()
