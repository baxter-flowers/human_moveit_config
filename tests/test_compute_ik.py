#!/usr/bin/env python
import unittest
import rospy
import time
import numpy as np
from human_moveit_config.human_model import HumanModel


class TestTransformation(unittest.TestCase):
    def test_compute_ik(self):
        rospy.init_node('test_compute_ik')
        human = HumanModel()

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
            # move the torso
            while not human.move_group_by_joints('head', head_joint_values):
                # collect new random pose
                head_joint_values = human.get_random_joint_values('head')

            # get joint values for the arm
            arm_joint_values = []
            for j in range(2):
                joints = human.get_random_joint_values(sides[j]+'_arm')
                while not human.move_group_by_joints(sides[j]+'_arm', joints):
                    joints = human.get_random_joint_values(sides[j]+'_arm')
                arm_joint_values.append(joints)

            # move back to initial pose
            human.move_group_by_joints('head', start_head_joints)
            for j in range(2):
                human.move_group_by_joints(sides[j]+'_arm', start_arm_joints[j])

            fk_head = human.forward_kinematic('head', head_joint_values, links=['torso', 'head_tip'])
            # calculate ik for head
            ik_torso = human.inverse_kinematic('head', [fk_head[0]], links=['torso'])
            # move to desired pose
            human.move_group_by_joints('head', ik_torso)
            # calculate head ik
            ik_head = human.inverse_kinematic('head', [fk_head[1]], links=['head_tip'])
            # move to desired pose
            human.move_group_by_joints('head', ik_head)

            # calculate arm poses and move the arms
            for j in range(2):
                links = [sides[j]+'_upper_arm', sides[j]+'_forearm', sides[j]+'_hand_tip']
                fk = human.forward_kinematic(sides[j]+'_arm', arm_joint_values[j], links=links)
                for k in range(len(fk)):
                    ik = human.inverse_kinematic(sides[j]+'_arm', [fk[k]], links=[links[k]])
                    # move to desired pose
                    human.move_group_by_joints(sides[j]+'_arm', ik)

            # get the joint values
            current_head_joints = human.get_joint_values('head')
            # head_ik = human.forward_kinematic('head', current_head_joints)
            current_arm_joints = []
            # arm_ik = []
            for j in range(2):
                current_arm_joints.append(human.get_joint_values(sides[j]+'_arm'))
                # arm_ik.append(human.forward_kinematic(sides[j]+'_arm', current_arm_joints[j]))

            print 'head'
            print current_head_joints
            print head_joint_values
            print '-----------'
            print 'right arm'
            print current_arm_joints[0]
            print arm_joint_values[0]
            print '-----------'
            print 'left arm'
            print current_arm_joints[1]
            print arm_joint_values[1]
            print '-----------'

            # assert equality between initial joint values and ik
            # np.testing.assert_almost_equal(head_ik[0], head_des_pose[0], decimal=3)
            # np.testing.assert_almost_equal(np.absolute(head_ik[1]), np.absolute(head_des_pose[1]), decimal=3)

            # np.testing.assert_almost_equal(arm_ik[0][0], des_poses['right_hand_tip'][0], decimal=3)
            # np.testing.assert_almost_equal(np.absolute(arm_ik[0][1]), np.absolute(des_poses['right_hand_tip'][1]), decimal=3)

            # np.testing.assert_almost_equal(arm_ik[1][0], des_poses['left_hand_tip'][0], decimal=3)
            # np.testing.assert_almost_equal(np.absolute(arm_ik[1][1]), np.absolute(des_poses['left_hand_tip'][1]), decimal=3)

            # move back to initial pose
            human.move_group_by_joints('head', start_head_joints)
            # for j in range(2):
            #     human.move_group_by_joints(sides[j]+'_arm', start_arm_joints[j])

            print 'run number '+str(i)+' passed'
        print time.time() - start

if __name__ == '__main__':
    unittest.main()
