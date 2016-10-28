#!/usr/bin/env python
# from human_moveit_config.human_model import HumanModel
import numpy as np
import scipy.optimize as opti
import transformations


class SensorCalibrator(object):
    def __init__(self):
        # create human model
        self.human = None
        self.ground_axis = []
        self.robot_axis = []
        self.fk = {}

    def set_ground_axis(self, axis):
        self.ground_axis = axis

    def set_robot_axis(self, axis):
        self.robot_axis = axis

    def extract_transforms(self, flat_transforms):
            # a transform is 3 pos and 4 rot
            nb_transform = len(flat_transforms) / 7
            list_transforms = []
            for i in range(nb_transform):
                pose = []
                # extract the pose
                pose.append(flat_transforms[i * 7:i * 7 + 3])
                pose.append(flat_transforms[i * 7 + 3:i * 7 + 7])
                # append it to the list of transforms
                list_transforms.append(pose)
            return list_transforms

    def _evaluate_calibration(self, calibrations):
        def quaternion_cost(norm_coeff):
            C = 0
            for transform in list_calibr:
                # norm of a quaternion is always 1
                C += norm_coeff * abs(np.linalg.norm(transform[1]) - 1)
            return C

        def distance_cost(pose1, pose2, rot_coeff=2):
            pos_cost = 0
            # calculate position ditance
            pos_cost = np.linalg.norm(np.array(pose1[0]) - np.array(pose2[0]))
            # distance between two quaternions
            rot_cost = 1 - np.inner(pose1[1], pose2[1])**2
            return pos_cost + rot_coeff * rot_cost

        def base_cost(base_pose, coeff=2):
            q = base_pose[1]
            # calculate z axis from quaternion
            z = [2 * (q[0] * q[2] + q[1] * q[3]),
                 2 * (q[1] * q[2] - q[0] * q[3]),
                 1 - 2 * q[0] * q[0] - 2 * q[1] * q[1]]
            # check that the z base axis is normal to the ground
            cost = coeff * abs(np.dot(z, self.ground_axis) - 1)
            return cost

        # first extract the transformations
        list_calibr = self.extract_transforms(calibrations)
        # collect the cost based on the quaternions
        cost = quaternion_cost(0.5)
        # set the base transform
        base_transform = list_calibr[0]
        inv_base = transformations.inverse_transform(base_transform)
        # get pose of the base
        base_pose = transformations.multiply_transform(self.recorded_poses[self.human.prefix + '/base'], inv_base)
        # calculate cost of the base
        cost += (10 * base_cost(base_pose))
        # loop trough all the transforms
        for i in range(1, len(list_calibr)):
            key = self.keys[i]
            # get the fk
            fk = self.fk[key]
            # compute the corresponding transformation from recorded data
            pose = transformations.multiply_transform(base_transform, self.recorded_poses[key])
            pose = transformations.multiply_transform(pose, transformations.inverse_transform(list_calibr[i]))
            pose[1] /= np.linalg.norm(pose[1])
            # compute the cost based on the distance
            cost += distance_cost(fk, pose)

        print cost
        return cost

    def calibrate(self, record, frames='all', maxiter=1000):
        def random_transforms(pos_bounds, rot_bounds):
            flat_transforms = []
            for key in self.keys:
                # add a random position within bounds
                # flat_transforms += np.random.uniform(pos_bounds[0], pos_bounds[1], 3).tolist()
                flat_transforms += [0, 0, 0]
                # add a random quaternion
                rot = np.random.uniform(rot_bounds[0], rot_bounds[1], 4)
                flat_transforms += (rot / np.linalg.norm(rot)).tolist()
            return flat_transforms

        def correct_base_guess(base_transform):
            if base_transform == 0:
                return False

            inv_base = transformations.inverse_transform(base_transform)
            q = inv_base[1]
            # calculate x base axis from quaternion
            x = [1 - 2 * q[1] * q[1] - 2 * q[2] * q[2],
                 2 * (q[0] * q[1] + q[2] * q[3]),
                 2 * (q[0] * q[2] - q[1] * q[3])]
            # check that the base is facing the robot (cos < 0)
            epsilon = -0.1
            return np.dot(x, self.robot_axis) < epsilon

        self.recorded_poses = record
        # calculate the fk of the human model in T pose
        js = self.human.get_initial_state()
        self.fk = self.human.forward_kinematic(js, links='all')
        if frames == 'all':
            self.keys = record.keys()
            # remove the hip from the list to put in first position
            self.keys.remove(self.human.prefix + '/base')
        else:
            self.keys = frames
        self.keys = [self.human.prefix + '/base'] + self.keys
        # set limits for search space
        bounds = []
        pos_bounds = [-0.1, 0.1]
        rot_bounds = [-1, 1]
        for key in self.keys:
            if key == self.human.prefix + '/base':
                bounds.append([0.05, 0.2])
                bounds.append(pos_bounds)
                bounds.append([-0.01, 0.01])
            elif key == self.human.prefix + '/shoulder_center':
                bounds.append([0.05, 0.2])
                bounds.append(pos_bounds)
                bounds.append(pos_bounds)
            elif key == self.human.prefix + '/head':
                bounds.append([0.05, 0.2])
                bounds.append(pos_bounds)
                bounds.append([0.05, 0.2])
            else:
                for i in range(3):
                    bounds.append(pos_bounds)
            for i in range(4):
                bounds.append(rot_bounds)
        # collect initial guess
        base_guess = 0
        while not correct_base_guess(base_guess):
            initial_calibr = random_transforms(pos_bounds, rot_bounds)
            base_guess = [initial_calibr[0:3], initial_calibr[3:7]]

        # find the optimized calibration
        res = opti.minimize(self._evaluate_calibration,
                            initial_calibr,
                            bounds=bounds,
                            method='L-BFGS-B')
                            # options={'maxfun': maxiter})

        # extract the list of transformations
        list_transforms = self.extract_transforms(res.x)
        res_calibr = {}
        for i in range(len(self.keys)):
            pos = list_transforms[i][0].tolist()
            rot = list_transforms[i][1].tolist()
            inv_trans = transformations.inverse_transform([pos, rot])
            res_calibr[self.keys[i]] = [list(inv_trans[0]), list(inv_trans[1])]
        print res
        return res_calibr
