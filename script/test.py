#!/usr/bin/env python
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


def compute_fk_client(self, group, joint_values, links):
        rospy.wait_for_service('compute_fk')
        try:
            compute_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = group.get_pose_reference_frame()

            rs = RobotState()
            rs.joint_state.header = header
            rs.joint_state.name = group.get_active_joints()
            rs.joint_state.position = joint_values

            res = compute_fk(header, links, rs)

            return res.pose_stamped
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


def main():
    robot = moveit_commander.RobotCommander()
    head_group = moveit_commander.MoveGroupCommander('head')
    r_arm_group = moveit_commander.MoveGroupCommander('right_arm')
    l_arm_group = moveit_commander.MoveGroupCommander('left_arm')
    opti = Optimizer()

    start_hand_pose = head_group.get_current_joint_values()
    start_r_pose = r_arm_group.get_current_joint_values()
    start_l_pose = l_arm_group.get_current_joint_values()

    start = time.time()
    for i in range(5):
        # get random pose
        head_joint_values = head_group.get_random_joint_values()
        r_joint_values = r_arm_group.get_random_joint_values()
        l_joint_values = l_arm_group.get_random_joint_values()
        # get fk
        head_des_pose = compute_fk_client(head_group, head_joint_values, ['head_tip'])
        r_des_pose = compute_fk_client(r_arm_group, r_joint_values, ['right_hand_tip'])
        l_des_pose = compute_fk_client(l_arm_group, l_joint_values, ['left_hand_tip'])
        # compute the ik
        
        
    print time.time() - start

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('test')
    main()
