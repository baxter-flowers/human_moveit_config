#!/usr/bin/env python
import numpy as np
import scipy.optimize as opti
from .human_model import HumanModel
import math
import transformations
import rospkg
import json


class OptitrackCalibrator(object):
    def __init__(self):
        # create human model
        self.human = HumanModel()
        # get initial guess for calibration
        rospack = rospkg.RosPack()
        with open(rospack.get_path("human_moveit_config")+"/config/calibration.json") as data_file:
            self.init_calibration = json.load(data_file)
        self.links = [['head', 'torso'], ['head', 'head_tip'],
                      ['right_arm', 'right_upper_arm'], ['right_arm', 'right_forearm'],
                      ['right_arm', 'right_hand_tip']]
        self.fk = {}

    def extract_transforms(self, flat_transforms):
            # a transform is 3 pos and 4 rot
            nb_transform = len(flat_transforms)/7
            list_transforms = []
            for i in range(nb_transform):
                pose = []
                # extract the pose
                pose.append(flat_transforms[i*7:i*7+3])
                pose.append(flat_transforms[i*7+3:i*7+7])
                # append it to the list of transforms
                list_transforms.append(pose)
            return list_transforms

    def _evaluate_calibration(self, calibrations):
        def quaternion_cost(norm_coeff):
            C = 0
            for transform in list_calibr:
                # norm of a quaternion is always 1
                C += norm_coeff*abs(np.linalg.norm(transform[1])-1)
            return C

        def distance_cost(pose1, pose2):
            # calculate position ditance
            pos_cost = np.linalg.norm(np.array(pose1[0])-np.array(pose2[0]))
            # distance between two quaternions
            try:
                rot_cost = math.acos(2*np.inner(pose1[1], pose2[1])**2-1)
            except ValueError:
                rot_cost = math.pi
            return pos_cost + rot_cost

        # first extract the transformations
        list_calibr = self.extract_transforms(calibrations)
        # collect the cost based on the quaternions
        cost = quaternion_cost(1)
        # set the hip transform
        hip_transform = list_calibr[0]
        # loop trough all the transforms
        for i in range(1, len(list_calibr)):
            # get the fk
            fk = self.fk[self.links[i-1][1]]
            # compute the corresponding transformation from recorded data
            pose = transformations.multiply_transform(hip_transform, self.recorded_poses[i])
            pose = transformations.multiply_transform(pose, transformations.inverse_transform(list_calibr[i]))
            # compute the cost based on the distance
            cost += distance_cost(fk, pose)

        print cost
        return cost

    def calibrate(self, record, joint_values):
        def flatten_transforms(transform_dict):
            flat_transforms = []
            for key, transform in transform_dict.iteritems():
                flat_transforms += transform[0]
                flat_transforms += transform[1]
            return flat_transforms

        self.recorded_poses = record
        # calculate the fk of the recorded pose
        for i in range(len(self.links)):
            group_name = self.links[i][0]
            # get the forward kinematic of the corresponding link
            self.fk[self.links[i][1]] = self.human.forward_kinematic(group_name, joint_values[group_name], links=self.links[i][1])

        # collect initial guess
        initial_calibr = flatten_transforms(self.init_calibration)
        # set limits for search space
        bounds = []
        pos_bounds = [-0.5, 0.5]
        rot_bounds = [-1, 1]
        for key, value in self.init_calibration.iteritems():
            for i in range(3):
                bounds.append(pos_bounds)
            for i in range(4):
                bounds.append(rot_bounds)

        # find the optimized calibration
        res = opti.minimize(self._evaluate_calibration,
                            initial_calibr,
                            bounds=bounds,
                            method='L-BFGS-B',
                            options={'maxiter': 500})

        # extract the list of transformations
        list_transforms = self.extract_transforms(res.x)
        res_calibr = {}
        res_calibr['/human/hip'] = list_transforms[0]
        res_calibr['/human/torso'] = list_transforms[1]
        res_calibr['/human/head'] = list_transforms[2]
        res_calibr['/human/right_shoulder'] = list_transforms[3]
        res_calibr['/human/right_wrist'] = list_transforms[4]
        res_calibr['/human/right_hand'] = list_transforms[5]
        print res_calibr

        return res_calibr
