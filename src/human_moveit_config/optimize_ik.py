#!/usr/bin/env python
import numpy as np
from scipy.optimize import minimize
from .human_model import HumanModel
from human_moveit_config.srv import GetHumanIKResponse
import transformations
from sensor_msgs.msg import JointState


class IKOptimizer:
    def __init__(self):
        self.model = HumanModel()
        # set the cost factors (end_effectors, fixed_joints)
        self.cost_factors = [1, 1]
        self.distance_factor = [0, 1]

    def fixed_joints_cost(self, joint_array, dict_values):
        cost = 0
        for key, value in dict_values.iteritems():
            if key in self.joint_names:
                cost += (joint_array[self.joint_names.index(key)] - value)**2
        return cost

    # def jacobian_fixed_joints_cost(self, joint_array, dict_values):
    #     jac_fix = np.zeros(len(self.model.get_joint_names()))
    #     for key, value in dict_values.iteritems():
    #         index = self.model.get_joint_names().index(key)
    #         jac_fix[index] = 2 * (joint_array[index] - value)
    #     return jac_fix

    def desired_poses_cost(self, joint_array, dict_values):
        # get the forward kinematic of the whole body at specified links
        js = JointState()
        js.name = self.joint_names
        js.position = joint_array
        fk = self.model.forward_kinematic(js, links=dict_values.keys())
        # loop through all the current poses and compare it with desired ones
        cost = 0
        for key, value in fk.iteritems():
            # extract the desired pose
            des_pose = dict_values[key]
            # calcualte distance in position
            cost += self.distance_factor[0] * np.sum((np.array(value[0]) - np.array(des_pose[0]))**2)
            # calcualte distance in rotation
            # cost += self.distance_factor[1]*2*np.arccos(abs(np.inner(value[1], des_pose[1])))
            cost += self.distance_factor[1] * (1 - np.inner(value[1], des_pose[1])**2)
        return cost

    # def quaternion_jacobian(self, quat_fk, jac_vect):
    #     # calculate the quaternion skew simetric matrix
    #     skew_matrix = [[-quat_fk[0], -quat_fk[1], -quat_fk[2]],
    #                    [quat_fk[3], -quat_fk[2], quat_fk[1]],
    #                    [quat_fk[2], quat_fk[3], -quat_fk[0]],
    #                    [-quat_fk[1], quat_fk[0], quat_fk[3]]]
    #     # multiply it with the jacobian
    #     jac_quat = 0.5 * np.dot(skew_matrix, jac_vect)

    #     # print jac_quat
    #     # # reorder it according to ROS order
    #     temp = jac_quat[0]
    #     jac_quat[:-1] = jac_quat[1:]
    #     jac_quat[-1] = temp

    #     # print jac_quat

    #     # print '--------------------'
    #     return jac_quat

    # def jacobian_desired_poses_cost(self, joint_array, dict_values):
    #     joint_names = self.model.get_joint_names()
    #     # get current state
    #     js = self.model.get_current_state()
    #     # set the new joint values
    #     js.position = joint_array
    #     fk = self.model.forward_kinematic(js, links=dict_values.keys())
    #     # loop through all the desired poses
    #     jac_des_pose = np.zeros(len(joint_names))
    #     for key, value in fk.iteritems():
    #         # get the group name corresponding to the link
    #         group_name = self.model.get_group_of_link(key)
    #         group_joints = self.model.get_joint_names(group_name)
    #         # extract the desired pose
    #         des_pose = dict_values[key]
    #         # calculate the jacobian for the given link
    #         jac_model = self.model.jacobian(group_name, js, use_quaternion=False, link=key)
    #         # precalculate difference in position
    #         pos_diff = np.array(value[0]) - np.array(des_pose[0])
    #         # pre calculate the inner product
    #         inner_prod = np.inner(value[1], des_pose[1])
    #         # flip the quaternion if necessary
    #         if inner_prod < 0:
    #             print "Hello Woooooooooooooorld"
    #             value[1] = -np.array(value[1])
    #             inner_prod = -inner_prod
    #         # quat_des = des_pose[1]
    #         for i in range(len(group_joints)):
    #             index = joint_names.index(group_joints[i])
    #             for j in range(3):
    #                 jac_des_pose[index] += 2 * self.distance_factor[0] * jac_model[j, i] * pos_diff[j]
    #             # extract the rotational part of the jacobian
    #             jac_vect = jac_model[3:, i]
    #             # convert it into quaternions
    #             jac_quat = self.quaternion_jacobian(value[1], jac_vect)
    #             # calculate jacobian of rotational distance
    #             jac_des_pose[index] += -2 * self.distance_factor[1] * np.inner(jac_quat, des_pose[1]) * inner_prod
    #     return jac_des_pose

    def cost_function(self, q, desired_poses={}, fixed_joints={}):
        cost = 0
        if desired_poses:
            cost += self.cost_factors[0] * self.desired_poses_cost(q, desired_poses)
        if fixed_joints:
            cost += self.cost_factors[1] * self.fixed_joints_cost(q, fixed_joints)
        return cost

    # def jacobian_cost_function(self, q, desired_poses={}, fixed_joints={}):
    #     jac_cost = np.zeros(len(self.model.get_joint_names()))
    #     if desired_poses:
    #         jac_cost += self.cost_factors[0] * self.jacobian_desired_poses_cost(q, desired_poses)
    #     if fixed_joints:
    #         jac_cost += self.cost_factors[1] * self.jacobian_fixed_joints_cost(q, fixed_joints)
    #     return jac_cost

    def handle_compute_ik(self, req):
        joint_names_set = set()
        # get the number of joints according to the required group
        for g in req.group_names:
            joint_names_set.update(self.model.get_joint_names(g))
        self.joint_names = list(joint_names_set)
        # convert the desired poses to dict
        desired_dict = {}
        for pose in req.desired_poses:
            desired_dict[pose.header.frame_id] = transformations.pose_to_list(pose)
        # convert the fixed joint state to dict
        fixed_joints_dict = {}
        for i in range(len(req.fixed_joints.name)):
            fixed_joints_dict[req.fixed_joints.name[i]] = req.fixed_joints.position[i]
        # get initial state
        init_state = req.seed
        # get only joints to optimize
        init_joints = []
        for name in self.joint_names:
            init_joints.append(init_state.position[init_state.name.index(name)])
        # get the joints limits for the optimization
        joint_limits = self.model.get_joint_limits(self.joint_names)
        # optimize to find the corresponding IK
        res = minimize(self.cost_function, init_joints,
                       # jac=self.jacobian_cost_function,
                       args=(desired_dict, fixed_joints_dict, ),
                       method='L-BFGS-B',
                       bounds=joint_limits)
        opt_joints = list(res.x)
        # convert it to a joint state
        js = self.model.get_current_state()
        js.position = list(js.position)
        js.name = list(js.name)
        # replace joint values with optimized ones
        for i in range(len(self.joint_names)):
            name = self.joint_names[i]
            js.position[js.name.index(name)] = opt_joints[i]
        # replace fixed values
        for key, value in fixed_joints_dict.iteritems():
            js.position[js.name.index(key)] = value
        # return server reply
        return GetHumanIKResponse(js)
