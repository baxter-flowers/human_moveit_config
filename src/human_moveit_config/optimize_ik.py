#!/usr/bin/env python
import numpy as np
import scipy.optimize as opti
from .human_model import HumanModel
import transformations
import math


class Criterion:
    def __init__(self):
        self.human = HumanModel()
        self.desired_poses = {}
        # initialize desired pose dict
        self.desired_poses['head_tip'] = []
        self.desired_poses['right_forearm'] = []
        self.desired_poses['left_forearm'] = []
        self.desired_poses['right_hand_tip'] = []
        self.desired_poses['left_hand_tip'] = []

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
                euler[2] = math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
            euler[1] = math.asin(2*(q[0]*q[2]-q[3]*q[1]))
            return euler
        pos = pose[0]
        rot = quaternion_to_euler(pose[1])
        return np.concatenate((pos, rot))

    def _extract_transforms(self, transforms):
        # a transform is 3 pos and 4 rot
        nb_transform = len(transforms)/7
        list_transforms = []
        for i in range(nb_transform):
            pose = []
            # extract the pose
            pose.append(transforms[i*7:i*7+3])
            pose.append(transforms[i*7+3:i*7+7])
            # convert the pose to mat
            list_transforms.append(transformations.list_to_m4x4(pose))
        return list_transforms

    def pose_distance(self, pose1, pose2):
        def squared_dist(x, y):
            return np.sum((x-y)**2)
        eul1 = self.pose_to_euler(pose1)
        eul2 = self.pose_to_euler(pose2)
        return squared_dist(eul1, eul2)

    def evaluate(self, joints, group_name, links=None):
        # calculate the distance with the desired pose
        pose = self.human.forward_kinematic(group_name, joints, links)
        if (links is None) or (len(links) == 1):  # TODO consider case where link is not eef
            eef_name = self.human.end_effectors[group_name]
            C = self.pose_distance(pose, self.desired_poses[eef_name])
        else:
            C = 0
            for i in range(len(links)):
                # only add the cost if the desired pose exist
                if self.desired_poses[links[i]]:
                    # calculate the score
                    C += self.pose_distance(pose[i], self.desired_poses[links[i]])
        # return the value of the criterion
        return C


class Optimizer:
    def __init__(self):
        self.criterion = Criterion()

    def _reset_groups_dict(self):
        self.groups_ik = {}
        # initialize the groups dict
        for key_group, groups in self.criterion.human.groups.iteritems():
            self.groups_ik[key_group] = []

    def _calculate_ik_by_group(self, group_name, links, move=True, tol=0.001):
        def solution_reached(group_name, joints, desired_pose):
            # compute fk of joint
            reached_pose = self.criterion.human.forward_kinematic(group_name, joints)
            dist = self.criterion.pose_distance(reached_pose, desired_pose)
            return dist < tol

        # initialize joints
        joints = self.criterion.human.get_random_joint_values(group_name)
        # get enf_effector name
        eef_name = self.criterion.human.end_effectors[group_name]
        # loop until convenient solution is reached
        while not solution_reached(group_name, joints, self.criterion.desired_poses[eef_name]):
            joints = self.criterion.human.get_random_joint_values(group_name)
            res = opti.minimize(self.criterion.evaluate,
                                joints,
                                args=(group_name, links),
                                bounds=self.criterion.human.joint_limits[group_name],
                                method='L-BFGS-B')
            joints = res.x.tolist()
        if move:
            # move to the desired head pose
            self.criterion.human.move_group_by_joints(group_name, joints)

    def compute_ik(self, dict_poses):
        # reset the dict of groups
        self._reset_groups_dict()
        # fill it with the desired poses
        for key_poses, pose in dict_poses.iteritems():
            try:
                self.criterion.desired_poses[key_poses] = pose
            except:
                pass
            for key_group, links in self.criterion.human.group_links.iteritems():
                if key_poses in links:
                    self.groups_ik[key_group].append(key_poses)

        # call the ik for each groups
        for group_names, links in self.groups_ik.iteritems():
            if links:
                self._calculate_ik_by_group(group_names, links, tol=0.00001)
