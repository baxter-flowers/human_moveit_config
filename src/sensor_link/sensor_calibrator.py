#!/usr/bin/env python
# from human_moveit_config.human_model import HumanModel
import numpy as np
import scipy.optimize as opti
import math
import transformations


class SensorCalibrator(object):
    def __init__(self):
        # create human model
        self.human = None
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
            pos_cost = 0
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
        cost = quaternion_cost(0.5)
        # set the base transform
        base_transform = list_calibr[0]
        # loop trough all the transforms
        for i in range(1, len(list_calibr)):
            key = self.keys[i]
            # get the fk
            fk = self.fk[key]
            # compute the corresponding transformation from recorded data
            pose = transformations.multiply_transform(base_transform, self.recorded_poses[key])
            pose = transformations.multiply_transform(pose, list_calibr[i])
            # compute the cost based on the distance
            cost += distance_cost(fk, pose)

        print cost
        return cost

    def calibrate(self, record, frames='all'):
        def random_transforms(pos_bounds, rot_bounds):
            flat_transforms = []
            for key in self.keys:
                # add a random position within bounds
                # flat_transforms += np.random.uniform(pos_bounds[0], pos_bounds[1], 3).tolist()
                flat_transforms += [0, 0, 0]
                # add a random quaternion
                rot = np.random.uniform(rot_bounds[0], rot_bounds[1], 4)
                flat_transforms += (rot/np.linalg.norm(rot)).tolist()
            return flat_transforms

        self.recorded_poses = record
        # calculate the fk of the human model in T pose
        js = self.human.get_initial_state()
        self.fk = self.human.forward_kinematic(js, links='all')
        if frames == 'all':
            self.keys = record.keys()
            # remove the hip from the list to put in first position
            self.keys.remove('base')
            self.keys = ['base'] + self.keys
        else:
            self.keys = frames
        # set limits for search space
        bounds = []
        pos_bounds = [-0.05, 0.05]
        rot_bounds = [-1, 1]
        for key in self.keys:
            for i in range(3):
                bounds.append(pos_bounds)
            for i in range(4):
                bounds.append(rot_bounds)
        # collect initial guess
        initial_calibr = random_transforms(pos_bounds, rot_bounds)
        # find the optimized calibration
        res = opti.minimize(self._evaluate_calibration,
                            initial_calibr,
                            bounds=bounds,
                            method='L-BFGS-B')

        # extract the list of transformations
        list_transforms = self.extract_transforms(res.x)
        res_calibr = {}
        for i in range(len(self.keys)):
            pos = list_transforms[i][0].tolist()
            rot = list_transforms[i][1].tolist()
            res_calibr[self.keys[i]] = [pos, rot]
        # invert base transformation to get them all in the same order
        pose = transformations.inverse_transform(res_calibr['base'])
        res_calibr['base'] = [pose[0], pose[1].tolist()]
        return res_calibr
