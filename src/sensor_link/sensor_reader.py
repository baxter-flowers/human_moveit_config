#!/usr/bin/env python
import tf
import transformations
import rospy
import numpy as np
import json
import rospkg


class SensorReader(object):
    def __init__(self, calibrated=False):
        rospack = rospkg.RosPack()
        self.pkg_dir = rospack.get_path("human_moveit_config")
        self.tfl = tf.TransformListener()
        self.skel_data = {}
        self.lengths = {}
        self.calibrated = calibrated
        self.calibration = (self.calibration_matrices(rospy.get_param('/human/calibration'))
                            if self.calibrated else {})
        self.sensor_frames = {}
        self.sensors = ['opt', 'kinect']
        self.sensors_ref = {'opt': 'optitrack', 'kinect': 'kinect'}

        self.skeletons = {}
        for s in self.sensors:
            self.skeletons[s] = {}
        self.init_sensor_frames()

    def init_sensor_frames(self):
        self.sensor_frames['kinect'] = ['head', 'neck', 'shoulder_center', 'spine',
                                        'left_shoulder', 'left_elbow', 'left_wrist', 'left_hand',
                                        'right_shoulder', 'right_elbow', 'right_wrist', 'right_hand',
                                        'left_hip', 'left_knee', 'left_ankle', 'left_foot',
                                        'right_hip', 'right_knee', 'right_ankle', 'right_foot']
        self.sensor_frames['opt'] = ['head', 'shoulder_center',
                                     'left_elbow', 'left_hand',
                                     'right_elbow', 'right_hand']

    def set_sensor_frames(self, dict_frames):
        self.sensor_frames = dict_frames

    def copy_skeleton(self, sensor_key):
        for key, value in self.skel_data.iteritems():
            self.skeletons[sensor_key][key] = value[1]

    def get_skeleton(self, sensor_key):
        self.copy_skeleton(sensor_key)
        skel = self.skeletons[sensor_key]
        return skel

    def calibrate(self):
        self.skeletons = {}
        for s in self.sensors:
            self.skeletons[s] = {}
        # get all skeletons from sensors, one sensor at a time
        for s in self.sensors:
            if self.update_skeleton(sensors=s):
                self.copy_skeleton(s)
        # check kinect skeleton is visible to create the model
        if self.skeletons['kinect']:
            self.generate_model_from_kinect()
            return True
        return False

    def calibration_matrices(self, d):
        mat_dict = {}
        self.calibrated_frames_set = {}
        for sensor, dico in d.iteritems():
            mat_dict[sensor] = {}
            self.calibrated_frames_set[sensor] = set()
            for key, value in dico.iteritems():
                mat_dict[sensor][key] = value
                mat_dict[sensor][key + '/inv'] = transformations.inverse_transform(value)
                self.calibrated_frames_set[sensor].add(key)
        return mat_dict

    def generate_model_from_kinect(self):
        kinect_data = self.skeletons['kinect']
        # extract frame positions
        p_spine = np.array(kinect_data['spine'][0])
        p_neck = np.array(kinect_data['neck'][0])
        p_head = np.array(kinect_data['head'][0])
        p_shoulder_center = np.array(kinect_data['shoulder_center'][0])
        p_shoulder = np.array(kinect_data['right_shoulder'][0])
        p_elbow = np.array(kinect_data['right_elbow'][0])
        p_wrist = np.array(kinect_data['right_wrist'][0])
        p_hip = np.array(kinect_data['right_hip'][0])
        p_knee = np.array(kinect_data['right_knee'][0])
        p_ankle = np.array(kinect_data['right_ankle'][0])
        # calculate the length of the model
        upper_arm_l = float(np.linalg.norm(p_elbow - p_shoulder))
        forearm_l = float(np.linalg.norm(p_wrist - p_elbow))
        torso_l = float(np.linalg.norm(p_neck - p_shoulder_center))
        spine_up_l = float(np.linalg.norm(p_shoulder_center - p_spine))
        spine_down_l = float(np.linalg.norm(p_spine))
        neck_l = float(np.linalg.norm(p_head - p_neck))
        thigh_l = float(np.linalg.norm(p_knee - p_hip))
        shin_l = float(np.linalg.norm(p_ankle - p_knee))
        shoulder_offset_w = float((p_shoulder - p_shoulder_center)[0])
        shoulder_offset_h = float((p_shoulder - p_shoulder_center)[1])
        hip_offset_w = float(p_hip[0])
        hip_offset_h = float(abs(p_hip[1]))
        # set the disct of lengths for the urdf model
        self.lengths = {'upper_arm_length': upper_arm_l,
                        'forearm_length': forearm_l,
                        'torso_length': torso_l,
                        'spine_up_length': spine_up_l,
                        'spine_down_length': spine_down_l,
                        'neck_length': neck_l,
                        'thigh_length': thigh_l,
                        'shin_length': shin_l,
                        'shoulder_offset_width': shoulder_offset_w,
                        'shoulder_offset_height': shoulder_offset_h,
                        'hip_offset_height': hip_offset_h,
                        'hip_offset_width': hip_offset_w,
                        'hand_length': 0.1,    # TODO
                        'foot_length': 0.1,    # TODO
                        'spine_down_radius': shoulder_offset_w - 0.04,
                        'spine_up_radius': shoulder_offset_w - 0.02,  # TODO
                        'torso_radius': shoulder_offset_w,  # TODO
                        'neck_radius': 0.04,  # TODO
                        'head_radius': 0.1,
                        'thigh_radius': 0.05}    # TODO
        # write the length file
        with open(self.pkg_dir + '/tmp/human_length.json', 'w') as outfile:
            json.dump(self.lengths, outfile, sort_keys=True, indent=4)
        # set the lengths on the parameter server
        rospy.set_param('/human/lengths', self.lengths)

    def update_skeleton(self, sensors='all', debug=False):
        def update_frame(target, prefix):
            base = 'base'
            # loop through all the prefixes
            for b_pref in prefix:
                for t_pref in prefix:
                    # check visibility of frame
                    if self.tfl.canTransform('/'+b_pref+'/human/'+base, '/'+t_pref+'/human/'+target, rospy.Time(0)):
                        time = self.tfl.getLatestCommonTime('/'+b_pref+'/human/'+base, '/'+t_pref+'/human/'+target)
                        if rospy.Time.now() - time < rospy.Duration(0.5):
                            frame = self.tfl.lookupTransform('/'+b_pref+'/human/'+base,
                                                             '/'+t_pref+'/human/'+target,
                                                             time)
                            # multiply each transformation by the calibration matrix
                            if self.calibrated:
                                dot_prod = transformations.multiply_transform(self.calibration[b_pref][base+'/inv'],
                                                                              frame)
                                dot_prod = transformations.multiply_transform(dot_prod,
                                                                              self.calibration[t_pref][target])
                                self.skel_data[target] = [t_pref, dot_prod]
                            else:
                                self.skel_data[target] = [t_pref, frame]
                            return True
            if debug:
                rospy.logwarn(target + ' not visible')
            return False

        def update_base_frame(prefix):
            for pref in prefix:
                ref = '/' + self.sensors_ref[pref] + '_frame'
                if self.tfl.canTransform(ref, '/' + pref + '/human/base', rospy.Time(0)):
                    time = self.tfl.getLatestCommonTime(ref, '/' + pref + '/human/base')
                    if rospy.Time.now() - time < rospy.Duration(0.5):
                        frame = self.tfl.lookupTransform(ref,
                                                         '/' + pref + '/human/base',
                                                         time)
                        # multiply with the calibration
                        if self.calibrated:
                            dot_prod = transformations.multiply_transform(frame,
                                                                          self.calibration[pref]['base'])
                            self.skel_data['base'] = [pref, dot_prod]
                        else:
                            self.skel_data['base'] = [pref, frame]
                        return True
            if debug:
                rospy.logwarn('base not visible')
            return False

        visible = True
        # flush the old skeleton
        self.skel_data = {}
        # check sensor usage
        if sensors is not list:
            if sensors == 'all':
                # use all possible sensors
                sensors = self.sensors
            else:
                sensors = [sensors]
        # get the union of frames based on the sensors
        set_frames = set()
        for s in sensors:
            set_frames.update(self.sensor_frames[s])
            # get the intersection with the calibrated matrices
            if self.calibrated:
                set_frames = set.intersection(set_frames, self.calibrated_frames_set[s])
        # loop through all the possible frames
        for frame in set_frames:
            frame_visible = update_frame(frame, prefix=sensors)
            visible = (visible and frame_visible)
        # update base pose
        base_visible = update_base_frame(prefix=sensors)
        visible = (visible and base_visible)
        return visible
