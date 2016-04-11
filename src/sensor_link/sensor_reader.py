#!/usr/bin/env python
import tf
import transformations
import rospy
import rospkg
import numpy as np


class SensorReader(object):
    def __init__(self, calibrated=False):
        self.rospack = rospkg.RosPack()
        self.tfl = tf.TransformListener()
        self.skel_data = {}
        self.lengths = {}
        self.calibrated = calibrated
        self.calibration = (self.calibration_matrices(rospy.get_param('/human/calibration'))
                            if self.calibrated else {})
        self.sensor_frames = {}
        self.sensors = ['optitrack', 'kinect']
        self.skeletons = {}
        for s in self.sensors:
            self.skeletons[s] = {}
        self.init_sensor_frames()

    def init_sensor_frames(self):
        self.sensor_frames['kinect'] = ['head_tip', 'neck', 'torso', 'waist',
                                        'left_shoulder', 'left_elbow', 'left_wrist', 'left_hand_tip',
                                        'right_shoulder', 'right_elbow', 'right_wrist', 'right_hand_tip',
                                        'left_hip', 'left_knee', 'left_ankle', 'left_foot_tip',
                                        'right_hip', 'right_knee', 'right_ankle', 'right_foot_tip']
        self.sensor_frames['optitrack'] = ['head_tip', 'torso', 'waist',
                                           'left_shoulder', 'left_elbow', 'left_hand_tip',
                                           'right_shoulder', 'right_elbow', 'right_hand_tip']

    def copy_skeleton(self, sensor_key):
        for key, value in self.skel_data.iteritems():
            self.skeletons[sensor_key][key] = value[1]

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
        for namespace, dico in d.iteritems():
            mat_dict[namespace] = {}
            for key, value in dico.iteritems():
                mat_dict[namespace][key] = value
                mat_dict[namespace][key+'/inv'] = transformations.inverse_transform(value)
        return mat_dict

    def generate_model_from_kinect(self):
        kinect_data = self.skeletons['kinect']
        # extract frame positions
        p_elbow = np.array(kinect_data['right_elbow'][0])
        p_wrist = np.array(kinect_data['right_wrist'][0])
        p_neck = np.array(kinect_data['neck'][0])
        p_torso = np.array(kinect_data['torso'][0])
        p_head = np.array(kinect_data['head_tip'][0])
        p_knee = np.array(kinect_data['right_knee'][0])
        p_ankle = np.array(kinect_data['right_ankle'][0])
        p_r_shoulder = np.array(kinect_data['right_shoulder'][0])
        p_l_shoulder = np.array(kinect_data['left_shoulder'][0])
        p_r_hip = np.array(kinect_data['right_hip'][0])
        p_l_hip = np.array(kinect_data['left_hip'][0])
        p_mid_shoulder = (p_r_shoulder - p_l_shoulder)/2
        p_mid_hip = (p_r_hip - p_l_hip)/2
        # calculate the length of the model
        upper_arm_l = float(np.linalg.norm(p_elbow - p_r_shoulder))
        forearm_l = float(np.linalg.norm(p_wrist - p_elbow))
        torso_l = float(np.linalg.norm(p_neck - p_torso))
        waist_l = float(np.linalg.norm(p_torso))
        neck_l = float(np.linalg.norm(p_head - p_neck))
        thigh_l = float(np.linalg.norm(p_knee - p_r_hip))
        shin_l = float(np.linalg.norm(p_ankle - p_knee))
        shoulder_offset_h = float(np.linalg.norm(p_mid_shoulder - p_torso))
        shoulder_offset_w = float(np.linalg.norm(p_r_shoulder - p_mid_shoulder))
        hip_offset_h = float(-np.linalg.norm(p_mid_hip))
        hip_offset_w = float(np.linalg.norm(p_r_hip - p_mid_hip))
        # set the disct of lengths for the urdf model
        self.lengths = {'upper_arm_length': upper_arm_l,
                        'forearm_length': forearm_l,
                        'torso_length': torso_l,
                        'waist_length': waist_l,
                        'neck_length': neck_l,
                        'thigh_length': thigh_l,
                        'shin_length': shin_l,
                        'shoulder_offset_width': shoulder_offset_w,
                        'shoulder_offset_height': shoulder_offset_h,
                        'hip_offset_width': hip_offset_w,
                        'hip_offset_height': hip_offset_h,
                        'hand_length': 0.1,    # TODO
                        'foot_length': 0.1,    # TODO
                        'waist_radius': 0.12,  # TODO
                        'torso_radius': 0.15,  # TODO
                        'head_radius': 0.1}    # TODO

        print self.lengths
        print '---------------------'

        # set the lengths on the parameter server
        rospy.set_param('/kinect/human_lengths', self.lengths)

    def update_skeleton(self, sensors=None):
        def update_frame(target, prefix):
            base = 'waist'
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
                            self.skel_data[target] = [t_pref, frame]
                            # multiply each transformation by the calibration matrix
                            if self.calibrated:
                                dot_prod = transformations.multiply_transform(self.calibration[b_pref][base+'/inv'],
                                                                              frame)
                                dot_prod = transformations.multiply_transform(dot_prod,
                                                                              self.calibration[t_pref][target])
                                self.skel_data[target] = [t_pref, dot_prod]
                            return True
            return False

        def update_base_frame(prefix):
            for pref in prefix:
                if self.tfl.canTransform(pref+'_frame', pref+'/human/waist', rospy.Time(0)):
                    time = self.tfl.getLatestCommonTime('kinect_frame', pref+'/human/waist')
                    if rospy.Time.now() - time < rospy.Duration(0.5):
                        frame = self.tfl.lookupTransform('/'+pref+'_frame',
                                                         '/'+pref+'/human/waist',
                                                         time)
                        self.skel_data['waist'] = [pref, frame]
                        # multiply with the calibration
                        if self.calibrated:
                            dot_prod = transformations.multiply_transform(frame,
                                                                          self.calibration[pref]['waist'])
                            self.skel_data['waist'] = [pref, dot_prod]
                        return True
            return False

        visible = True
        # flush the old skeleton
        self.skel_data = {}
        # check sensor usage
        if sensors is not list:
            if sensors is 'all':
                # use all possible sensors
                sensors = self.sensors
            else:
                sensors = [sensors]
        # get the union of frames based on the sensors
        list_frames = []
        for s in sensors:
            list_frames += self.sensor_frames[s]
        list_frames = list(set(list_frames))
        # loop through all the possible frames
        for frame in list_frames:
            if frame is not 'waist':
                visible = (visible and update_frame(frame, prefix=sensors))
            else:
                visible = (visible and update_base_frame(prefix=sensors))
        return visible
