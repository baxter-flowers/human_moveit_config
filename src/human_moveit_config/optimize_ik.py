#!/usr/bin/env python
import numpy as np
import scipy.optimize as opti
from .human_model import HumanModel
import math
import rospy
from human_moveit_config.srv import GetHumanIKResponse
from sensor_msgs.msg import JointState
import transformations


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

    def pose_distance(self, pose1, euler_des_pose):
        def squared_dist(x, y):
            return np.sum((x-y)**2)
        eul1 = self.pose_to_euler(pose1)
        return squared_dist(eul1, euler_des_pose)

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
                C += self.pose_distance(pose[i], self.desired_poses[links[i]])
        return C

    def jacobian(self, joints, group_name, links=None):
        def dist_jacobian(jac, dist, joint_index):
            jac_dist = 0
            for d in range(len(dist)):
                jac_dist += jac[d, joint_index]*dist[d]
            jac_dist *= 2
            return jac_dist

        # print links

        jac_crit = np.zeros(len(joints))
        # calculate the forward kinematics of joints
        pose = self.human.forward_kinematic(group_name, joints, links=links)
        if links is None:
            links = [self.human.end_effectors[group_name]]
        if type(links) is not list:
            links = [links]
        # deal with pose not being a list
        if len(links) == 1:
            pose = [pose]
        # else case with multiple links
        for i in range(len(links)):
            # compute distance with reference frame
            if len(self.desired_poses[links[i]]) > 0:
                dist = self.pose_to_euler(pose[i]) - self.desired_poses[links[i]]
                # compute the human jacobian
                jac_human = self.human.jacobian(group_name, joints, links[i])
                # loop through all the joints to calculate the criterion jacobian
                for n in range(len(joints)):
                    # calculate the value based on the distance
                    jac_crit[n] += dist_jacobian(jac_human, dist, n)

            # print jac_human
            # print '-----------------------------'
        # print jac_crit
        return jac_crit


class Optimizer:
    def __init__(self):
        self.criterion = Criterion()

    def handle_compute_ik(self, req):
        def solution_reached(joints):
            bool_reached = True
            for link in req.links:
                # compute fk of joint
                reached_pose = self.criterion.human.forward_kinematic(req.group_name,
                                                                      joints,
                                                                      links=link,
                                                                      joint_names=req.active_joints)
                # convert the pose to euler
                eul_pose = self.criterion.pose_to_euler(reached_pose)
                desired_pose = self.criterion.desired_poses[link]
                dist = np.linalg.norm(eul_pose - desired_pose)

                print dist

                bool_reached = bool_reached and (dist < req.tolerance)
            return bool_reached

        print 'Received new IK request'
        # set the desired poses in euler
        for i in range(len(req.links)):
            pose = transformations.pose_to_list(req.desired_poses[i])
            self.criterion.desired_poses[req.links[i]] = self.criterion.pose_to_euler(pose)
        # initialize joints
        joints = self.criterion.human.get_random_joint_values(req.group_name, req.active_joints)
        joint_limits = self.criterion.human.joint_limits_by_group(req.group_name, req.active_joints)['limits']
        # loop until convenient solution is reached
        while not solution_reached(joints) and not rospy.is_shutdown():
            joints = self.criterion.human.get_random_joint_values(req.group_name, req.active_joints)
            res = opti.minimize(self.criterion.evaluate,
                                joints,
                                args=(req.group_name, req.active_joints, req.links),
                                # jac=self.criterion.jacobian,
                                bounds=joint_limits,
                                method='L-BFGS-B')
            joints = res.x.tolist()
        # set the result
        if req.active_joints:
            joint_result = self.criterion.human.set_joint_values(req.group_name, joints, req.active_joints)
        else:
            joint_result = joints
        # convert it to a joint state
        js = JointState()
        js.name = self.criterion.human.groups[req.group_name].get_active_joints()
        js.position = joint_result

        print'IK computed and sent back'
        # return server reply
        return GetHumanIKResponse(js)

    # def _reset_groups_dict(self):
    #     self.groups_ik = {}
    #     # initialize the groups dict
    #     for key_group, groups in self.criterion.human.groups.iteritems():
    #         self.groups_ik[key_group] = []

    # def _calculate_ik_by_group(self, group_name, start_index, links, move=True, tol=0.001):
    #     def solution_reached(group_name, joints):
    #         bool_reached = True
    #         for link in links:
    #             # compute fk of joint
    #             reached_pose = self.criterion.human.forward_kinematic(group_name, joints, link)
    #             eul_pose = self.criterion.pose_to_euler(reached_pose)
    #             desired_pose = self.criterion.desired_poses[link]
    #             dist = np.linalg.norm(eul_pose - desired_pose)
    #             bool_reached = bool_reached and (dist < tol)

    #             print dist
    #         return bool_reached

    #     if links is None:
    #         links = [self.human.end_effectors[group_name]]
    #     if type(links) is not list:
    #         links = [links]
    #     # initialize joints
    #     init_joints = self.criterion.human.get_joint_values(group_name)
    #     joints = copy.copy(init_joints)
    #     # loop until convenient solution is reached
    #     while not solution_reached(group_name, joints) and not rospy.is_shutdown():
    #         joints = copy.copy(init_joints)
    #         opt_joints = joints[start_index:]
    #         fixed_joints = joints[:start_index]
    #         res = opti.minimize(self.criterion.evaluate,
    #                             opt_joints,
    #                             args=(fixed_joints, group_name, links),
    #                             # jac=self.criterion.jacobian,
    #                             bounds=self.criterion.human.joint_limits[group_name][start_index:],
    #                             method='L-BFGS-B')
    #         joints = res.x.tolist()
    #     if move:
    #         # move to the desired head pose
    #         self.criterion.human.move_group_by_joints(group_name, joints)

    # def compute_ik(self, dict_poses):
    #     # reset the dict of groups
    #     self._reset_groups_dict()
    #     # fill it with the desired poses
    #     for key_poses, pose in dict_poses.iteritems():
    #         try:
    #             self.criterion.desired_poses[key_poses] = self.criterion.pose_to_euler(pose)
    #         except:
    #             pass
    #         for key_group, links in self.criterion.human.group_links.iteritems():
    #             if key_poses in links:
    #                 self.groups_ik[key_group].append(key_poses)

    #     print self.criterion.desired_poses

    #     self.groups_ik['head'] = ['torso', 'head_tip']
    #     self.groups_ik['right_arm'] = ['right_upper_arm', 'right_forearm', 'right_hand_tip']
    #     self.groups_ik['left_arm'] = ['left_upper_arm', 'left_forearm', 'left_hand_tip']

    #     # call the ik for each groups starting with the head
    #     if self.groups_ik['head']:
    #         temp_link = []
    #         # optimize link by link
    #         start = 0
    #         i = 0
    #         for link in self.groups_ik['head']:
    #             temp_link.append(link)
    #             self._calculate_ik_by_group('head', start, temp_link, tol=0.1)
    #             start += self.criterion.nb_joints_by_group['head'][i]
    #             i += 1
    #         # remove the head not to calculate it again
    #         self.groups_ik['head'] = []
    #     # calculate the other groups
    #     for group_names, links in self.groups_ik.iteritems():
    #         if links:
    #             temp_link = []
    #             start = 0
    #             i = 0
    #             # optimize link by link
    #             for link in links:
    #                 temp_link.append(link)
    #                 self._calculate_ik_by_group(group_names, start, temp_link, tol=0.1)
    #                 start += self.criterion.nb_joints_by_group[group_names][i]
    #                 i += 1
