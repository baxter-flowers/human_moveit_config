#!/usr/bin/env python
from .human_model import HumanModel
from human_moveit_config.srv import GetHumanIKResponse
from human_moveit_config.srv import GetHumanIK
import transformations
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
        for i, l in enumerate(self.links):
            self.joint_by_links[l] = self.model.get_joints_chain(l, self.bases[i])

        print self.joint_by_links

        self.lock = Lock()
        self.div_ik_srv = {}
        for l in self.links:
            rospy.wait_for_service('/ik/' + l)
            print 'ready ' + l
            self.div_ik_srv[l] = rospy.ServiceProxy('/ik/' + l, GetHumanIK)

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
            joint_state = res.joint_state
        except:
            return 1

        with self.lock:
            result[group] = {'joint_names': joint_state.name, 'joint_values': joint_state.position}

    def handle_compute_ik_divided(self, req):
        # convert the desired poses to dict
        desired_dict = {}
        for pose in req.desired_poses:
            if pose.header.frame_id in self.links:
                desired_dict[pose.header.frame_id] = transformations.pose_to_list(pose)
        nb_frames = len(desired_dict.keys())
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
            for name in self.joint_by_links[key]:
                js.position[js.name.index(name)] = value['joint_values'][value['joint_names'].index(name)]
        # replace fixed values
        for key, value in fixed_joints_dict.iteritems():
            js.position[js.name.index(key)] = value
        # return server reply
        return GetHumanIKResponse(js)
