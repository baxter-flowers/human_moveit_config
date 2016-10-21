#!/usr/bin/env python
import numpy as np
from scipy.optimize import minimize
from .human_model import HumanModel
from human_moveit_config.srv import GetHumanIKResponse
from human_moveit_config.srv import GetHumanIK
import transformations
from sensor_msgs.msg import JointState
from threading import Thread
from threading import Lock
import rospy


class IKOptimizer:
    def __init__(self):
        self.model = HumanModel()
        self.prefix = self.model.prefix
        # set the cost factors (end_effectors, fixed_joints)
        self.cost_factors = [1, 1]
        self.distance_factor = [1, 3]
        self.links = [self.prefix + '/shoulder_center',
                      self.prefix + '/head',
                      self.prefix + '/left_elbow',
                      self.prefix + '/left_hand',
                      self.prefix + '/right_elbow',
                      self.prefix + '/right_hand']

        self.bases = [self.prefix + '/base',
                      self.prefix + '/shoulder_center',
                      self.prefix + '/shoulder_center',
                      self.prefix + '/left_elbow',
                      self.prefix + '/shoulder_center',
                      self.prefix + '/right_elbow']

        self.joint_by_links = {}
        for l in self.links:
            self.joint_by_links[l] = self.model.joint_by_links[l]
        for s in ['right', 'left']:
            link = self.prefix + '/' + s + '_elbow'
            self.joint_by_links[link] = (self.model.joint_by_links[self.prefix + '/' + s + '_shoulder'] +
                                         self.joint_by_links[link])

        self.lock = Lock()
        self.div_ik_srv = {}
        for l in self.links:
            rospy.wait_for_service('/ik/' + l)
            print 'ready ' + l
            self.div_ik_srv[l] = rospy.ServiceProxy('/ik/' + l, GetHumanIK)

    def fixed_joints_cost(self, joint_array, dict_values):
        cost = 0
        for key, value in dict_values.iteritems():
            if key in self.joint_names:
                cost += (joint_array[self.joint_names.index(key)] - value)**2
        return cost

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
            cost += self.distance_factor[1] * (1 - np.inner(value[1], des_pose[1])**2)
        return cost

    def cost_function(self, q, desired_poses={}, fixed_joints={}):
        cost = 0
        if desired_poses:
            cost += self.cost_factors[0] * self.desired_poses_cost(q, desired_poses)
        if fixed_joints:
            cost += self.cost_factors[1] * self.fixed_joints_cost(q, fixed_joints)
        return cost

    def compute_sub_ik(self, group, desired_dict, result, tol=0.001):
        # get the desired pose in the correct base frame
        index = self.links.index(group)
        base = self.bases[index]
        tr = desired_dict[group]
        if base != self.prefix + '/base':
            if base in desired_dict.keys():
                tr_base = desired_dict[base]
                inv_base = transformations.inverse_transform(tr_base)
                desired_pose = transformations.multiply_transform(inv_base, tr)
            else:
                return 1
        else:
            desired_pose = tr

        # transform it back to PoseStamped
        desired_pose = transformations.list_to_pose(desired_pose)
        try:
            # call the srv
            res = self.div_ik_srv[group](desired_poses=[desired_pose], tolerance=tol)
            joints = res.joint_state.position
        except:
            return 1

        with self.lock:
            result[group] = joints

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

    def handle_compute_ik_divided(self, req):
        nb_frames = len(req.desired_poses)
        # convert the desired poses to dict
        desired_dict = {}
        for pose in req.desired_poses:
            desired_dict[pose.header.frame_id] = transformations.pose_to_list(pose)
        # convert the fixed joint state to dict
        fixed_joints_dict = {}
        for i in range(len(req.fixed_joints.name)):
            fixed_joints_dict[req.fixed_joints.name[i]] = req.fixed_joints.position[i]
        # create as many threads as nb frames
        threads = [None] * nb_frames
        results = {}
        for i, frame in enumerate(desired_dict.keys()):
            threads[i] = Thread(target=self.compute_sub_ik, args=(frame, desired_dict, results, req.tolerance))
            threads[i].start()
        # join thread results
        for i in range(len(threads)):
            threads[i].join()
        # convert restult to a joint state
        js = req.seed
        js.position = list(js.position)
        js.name = list(js.name)
        # replace joint values with optimized ones
        for key, value in results.iteritems():
            for i, name in enumerate(self.joint_by_links[key]):
                js.position[js.name.index(name)] = value[i]
        # replace fixed values
        for key, value in fixed_joints_dict.iteritems():
            js.position[js.name.index(key)] = value
        # return server reply
        return GetHumanIKResponse(js)
