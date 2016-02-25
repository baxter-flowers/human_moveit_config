#!/usr/bin/env python
import numpy as np
import scipy.optimize as opti
from .human_model import HumanModel
import math
import rospy
from human_moveit_config.srv import GetHumanIKResponse
from sensor_msgs.msg import JointState
import transformations
from copy import copy


class Criterion(object):
    def __init__(self):
        self.human = HumanModel()
        self.desired_poses = {}

    def pose_to_euler(self, pose):
        def quaternion_to_euler(q):
            euler = np.zeros(3)
            test_sing = q[0]*q[1] + q[2]*q[3]
            if test_sing > 0.499:
                euler[0] = 2*math.atan2(q[0], q[3])
                euler[1] = math.pi/2.
                euler[2] = 0
            elif test_sing < -0.499:
                euler[0] = -2*math.atan2(q[0], q[3])
                euler[1] = -math.pi/2.
                euler[2] = 0
            else:
                euler[0] = math.atan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]**2+q[2]**2))
                try:
                    euler[1] = math.asin(2*(q[0]*q[2]-q[3]*q[1]))
                except ValueError:
                    euler[1] = math.copysign(math.pi/2, 2*(q[0]*q[2]-q[3]*q[1]))
                euler[2] = math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
            return euler
        pos = pose[0]
        rot = quaternion_to_euler(pose[1])
        return np.concatenate((pos, rot))

    def distance_cost(self, pose1, pose2):
        # calculate position ditance
        pos_cost = np.linalg.norm(np.array(pose1[0])-np.array(pose2[0]))
        # distance between two quaternions
        # try:
        #     rot_cost = math.acos(2*np.inner(pose1[1], pose2[1])**2-1)
        # except ValueError:
        #     rot_cost = math.pi
        rot_cost = 1-np.inner(pose1[1], pose2[1])**2
        return pos_cost + rot_cost

    def evaluate(self, joints, group_name, joint_names, links):
        # calculate the distance with the desired pose
        pose = self.human.forward_kinematic(group_name, joints, links=links, joint_names=joint_names)
        # deal with pose not being a list
        if len(links) == 1:
            pose = [pose]
        C = 0
        for i in range(len(links)):
            # only add the cost if the desired pose exist
            if len(self.desired_poses[links[i]]) > 0:
                # calculate the score
                C += self.distance_cost(pose[i], self.desired_poses[links[i]])
        return C

    def jacobian(self, joints, group_name, joint_names, links):
        def pos_jacobian(jac, pos, des_pos, joint_index):
            crit = 0
            dist = np.linalg.norm(np.array(pos)-np.array(des_pos))
            diff = np.array(pos) - np.array(des_pos)
            for d in range(3):
                crit += jac[d, joint_index]*diff[d]
            if dist > 0.0001:
                crit /= dist
            else:
                crit = 0.0
            return crit

        # def rot_jacobian(jac, rot, des_rot, joint_index):
        #     # get the column of the jacobian for rotation at joint index
        #     jac_rot = jac[3:, joint_index]
        #     # the quaternion in the jacobian is in [w,x,y,z] order thus it is necessary to change it
        #     # rot_modified = np.concatenate(([rot[-1]], rot[:3]))
        #     # des_modified = np.concatenate(([des_rot[-1]], des_rot[:3]))
        #     # jac_inner = np.inner(jac_rot, des_modified)
        #     # rot_inner = np.inner(rot_modified, des_modified)
        #     jac_inner = np.inner(jac_rot, des_rot)
        #     rot_inner = np.inner(rot, des_rot)
        #     # apply chain rule of derivative
        #     if abs(1-rot_inner**2) > 0.0001:
        #         crit = -2*jac_inner/math.sqrt(1-rot_inner**2)
        #     else:
        #         crit = 0.0
        #     return crit
        def rot_jacobian(jac, rot, des_rot, joint_index):
            jac_rot = jac[3:, joint_index]
        #     # the quaternion in the jacobian is in [w,x,y,z] order thus it is necessary to change it
            rot_modified = np.concatenate(([rot[-1]], rot[:3]))
            des_modified = np.concatenate(([des_rot[-1]], des_rot[:3]))
            jac_inner = np.inner(jac_rot, des_modified)
            rot_inner = np.inner(rot_modified, des_modified)
            # jac_inner = np.inner(jac_rot, des_rot)
            # rot_inner = np.inner(rot, des_rot)
            crit = -2*jac_inner*rot_inner
            return crit

        jac_crit = np.zeros(len(joints))
        # calculate the distance with the desired pose
        pose = self.human.forward_kinematic(group_name, joints, links=links, joint_names=joint_names)
        # deal with pose not being a list
        if len(links) == 1:
            pose = [pose]
        # get the index of the joints
        joint_index = self.human.get_joint_index(group_name, joint_names)
        # else case with multiple links
        for i in range(len(links)):
            # compute distance with reference frame
            if len(self.desired_poses[links[i]]) > 0:
                # compute the human jacobian
                jac_human = self.human.jacobian(group_name, joints, use_quaternion=True, link=links[i])
                # loop through all the joints to calculate the criterion jacobian
                for n in range(len(joints)):
                    # calculate the value based on the distance in both position and orientation
                    jac_crit[n] += pos_jacobian(jac_human, pose[i][0], self.desired_poses[links[i]][0], joint_index[n])
                    jac_crit[n] += 10*rot_jacobian(jac_human, pose[i][1], self.desired_poses[links[i]][1], joint_index[n])
        # print jac_crit
        return jac_crit


class Optimizer:
    def __init__(self):
        self.criterion = Criterion()
        self.max_iter = 50

    def handle_compute_ik(self, req):
        def solution_reached(joints, previous_value=None):
            bool_reached = True
            sum_dist = 0
            for link in req.links:
                # compute fk of joint
                reached_pose = self.criterion.human.forward_kinematic(req.group_name,
                                                                      joints,
                                                                      links=link,
                                                                      joint_names=req.active_joints)
                # calculate the distance
                dist = self.criterion.distance_cost(reached_pose, self.criterion.desired_poses[link])
                sum_dist += dist
                bool_reached = bool_reached and (dist < req.tolerance)
            # check if the maximum number of iterations have been reached
            bool_reached = bool_reached or iteration > self.max_iter
            # check improvment with previous results
            if previous_value is not None:
                bool_reached = bool_reached or abs(sum_dist-previous_value) < 0.001
            return bool_reached, sum_dist

        print 'Received new IK request'
        # set the desired poses in euler
        for i in range(len(req.links)):
            pose = transformations.pose_to_list(req.desired_poses[i])
            self.criterion.desired_poses[req.links[i]] = pose
        # initialize joints
        init_joints = self.criterion.human.get_joint_values(req.group_name, req.active_joints)
        if req.active_joints:
            joint_limits = self.criterion.human.get_joint_limits(req.active_joints)
        else:
            joint_limits = self.criterion.human.joint_limits_by_group(req.group_name)['limits']
        # loop until convenient solution is reached
        iteration = 0
        min_crit = 1000
        joints = copy(init_joints)
        best_joints = copy(init_joints)
        # loop till a solution is not found
        solution_found, crit_value = solution_reached(joints)
        previous_value = 1000
        while not solution_found and not rospy.is_shutdown():
            if req.continuity:
                joints = copy(init_joints)
            else:
                joints = self.criterion.human.get_random_joint_values(req.group_name, req.active_joints)
            res = opti.minimize(self.criterion.evaluate,
                                joints,
                                args=(req.group_name, req.active_joints, req.links),
                                # jac=self.criterion.jacobian,
                                bounds=joint_limits,
                                method='L-BFGS-B')
            joints = res.x.tolist()
            solution_found, crit_value = solution_reached(joints, previous_value)
            # solution_found, crit_value = solution_reached(joints)
            print crit_value
            if crit_value < min_crit:
                best_joints = res.x.tolist()
                min_crit = crit_value
            iteration += 1
            previous_value = crit_value
        # set the result
        if req.active_joints:
            joint_result = self.criterion.human.set_joint_values(req.group_name, best_joints, req.active_joints)
        else:
            joint_result = best_joints
        # convert it to a joint state
        js = JointState()
        js.name = self.criterion.human.groups[req.group_name].get_active_joints()
        js.position = joint_result

        print'IK computed and sent back'
        # return server reply
        return GetHumanIKResponse(js)
