#!/usr/bin/env python
import numpy as np
import scipy.optimize as opti
from transformations import multiply_transform
from transformations import inverse_transform
from itertools import permutations


class SensorCalibrator(object):
    def __init__(self):
        # create human model
        self.human = None
        self.fk = {}
        self.calibrations = {}
        self.records = {}
        self.links = []
        self.ground_axis = []

    def set_ground_axis(self, axis):
        self.ground_axis = axis

    def reset_calibration(self):
        self.calibrations = {}
        for k in self.links:
            self.calibrations[k] = []

    def calculate_transforms(self, base_transform):
        for skel_pose, dict_pose in self.fk.iteritems():
            for key, value in dict_pose.iteritems():
                # calculate the transformation
                p = multiply_transform(base_transform, value)
                p = multiply_transform(self.records[skel_pose][self.human.prefix + '/base'], p)
                inv_frame = inverse_transform(self.records[skel_pose][key])
                p = multiply_transform(inv_frame, p)
                # add it to the list of transformation
                self.calibrations[key].append(p)

    def extract_transform(self, flat_transform):
            # a transform is 3 pos and 4 rot
            pose = []
            # extract the pose
            pose.append(flat_transform[:3].tolist())
            rot = np.array(flat_transform[3:])
            pose.append((rot / np.linalg.norm(rot)).tolist())
            return pose

    def _evaluate_calibration(self, q):
        def distance_cost(list_poses, rot_coeff=3.14159):
            pos_cost = 0
            rot_cost = 0
            # calculate the distance between all p2p permutations of the list
            perm_list = permutations(list_poses, 2)
            for p in perm_list:
                # calculate position ditance
                pos_cost = np.linalg.norm(np.array(p[0][0]) - np.array(p[1][0]))
                # distance between two quaternions
                rot_cost = 1 - np.inner(p[0][1], p[1][1])**2
            return pos_cost + rot_coeff * rot_cost

        def base_rotation_cost(base_rot, coeff=2):
            q = base_rot
            # calculate z axis from quaternion
            z = [2 * (q[0] * q[2] + q[1] * q[3]),
                 2 * (q[1] * q[2] - q[0] * q[3]),
                 1 - 2 * q[0] * q[0] - 2 * q[1] * q[1]]
            # check that the z base axis is normal to the ground
            cost = coeff * abs(np.dot(z, self.ground_axis) - 1)
            return cost
        # reset the calibration
        self.reset_calibration()
        # create the calibration from the optimized base pose
        base_transform = self.extract_transform(q)
        self.calculate_transforms(base_transform)
        # for all the calibration calculate the difference between the transformations
        cost = base_rotation_cost(base_transform[1])
        for key, transforms in self.calibrations.iteritems():
            cost += distance_cost(transforms)
        self.calibrations[self.human.prefix + '/base'] = [base_transform]
        print cost
        return cost

    def calibrate(self, record, joint_states, links, maxiter=1000):
        def random_transform(pos_bounds, rot_bounds):
            flat_transform = []
            flat_transform += np.random.uniform(pos_bounds[0], pos_bounds[1], 3).tolist()
            # add a random quaternion
            rot = np.random.uniform(rot_bounds[0], rot_bounds[1], 4)
            flat_transform += (rot / np.linalg.norm(rot)).tolist()
            return flat_transform
        self.records = record
        self.links = links
        for key, js in joint_states.iteritems():
            self.fk[key] = self.human.forward_kinematic(js, links=self.links)

        # set limits for search space
        bounds = []
        pos_bounds = [-1, 1]
        rot_bounds = [-1, 1]
        for i in range(3):
            bounds.append(pos_bounds)
        for i in range(4):
            bounds.append(rot_bounds)
        # get initial guess
        initial_calibr = random_transform(pos_bounds, rot_bounds)
        # find the optimized calibration
        res = opti.minimize(self._evaluate_calibration,
                            initial_calibr,
                            bounds=bounds,
                            method='L-BFGS-B')
                            # options={'maxfun': maxiter})
        print res

        print self.calibrations

        # res_calibr = {}
        # for l in links:
        #     res_calibr[l] = self.calibrations[self.human.prefix + "/" + l][0]
        # res_calibr["base"] = self.calibrations[self.human.prefix + "/base"][0]
        return self.calibrations
