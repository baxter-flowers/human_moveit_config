#!/usr/bin/env python
from tools.udp_link import UDPLink
import numpy as np


class UnityBridge(object):
    def __init__(self, ip="BAXTERFLOWERS.local", port=5005):
        self.udp = udp = UDPLink(ip, port)
        self.corresp_dict = {}
        self.base_vector = np.zeros(92)
        self.init_dict()

    def init_dict(self):
        # create the base vector that contains all the offsets
        self.base_vector[18] = 0.6
        self.base_vector[21] = 1.0
        self.base_vector[26] = 0.6
        self.base_vector[29] = 1.0
        self.base_vector[36] = 0.4
        self.base_vector[37] = 0.3
        self.base_vector[39] = 1.0
        self.base_vector[45] = 0.4
        self.base_vector[46] = 0.3
        self.base_vector[48] = 1.0
        self.base_vector[52] = -1.33
        self.base_vector[53] = -0.29
        self.base_vector[54] = 0.64
        self.base_vector[55] = 0.64
        self.base_vector[56] = 0.67
        self.base_vector[57] = -0.46
        self.base_vector[58] = 0.81
        self.base_vector[59] = 0.81
        self.base_vector[60] = 0.67
        self.base_vector[61] = -0.61
        self.base_vector[62] = 0.81
        self.base_vector[63] = 0.81
        self.base_vector[64] = 0.67
        self.base_vector[65] = -0.61
        self.base_vector[66] = 0.81
        self.base_vector[67] = 0.81
        self.base_vector[68] = 0.67
        self.base_vector[69] = -0.46
        self.base_vector[70] = 0.81
        self.base_vector[71] = 0.81
        self.base_vector[72] = -1.33
        self.base_vector[73] = -0.29
        self.base_vector[74] = 0.64
        self.base_vector[75] = 0.64
        self.base_vector[76] = 0.67
        self.base_vector[77] = -0.46
        self.base_vector[78] = 0.81
        self.base_vector[79] = 0.81
        self.base_vector[80] = 0.67
        self.base_vector[81] = -0.61
        self.base_vector[82] = 0.81
        self.base_vector[83] = 0.81
        self.base_vector[84] = 0.67
        self.base_vector[85] = -0.61
        self.base_vector[86] = 0.81
        self.base_vector[87] = 0.81
        self.base_vector[88] = 0.67
        self.base_vector[89] = -0.46
        self.base_vector[90] = 0.81
        self.base_vector[91] = 0.81
        # create the dict for each joints with change of sign and vector index
        self.corresp_dict['spine_0'] = [1, 1]
        self.corresp_dict['spine_1'] = [0, 1]
        self.corresp_dict['spine_2'] = [2, 1]

        self.corresp_dict['neck_0'] = [7, 1]
        self.corresp_dict['neck_1'] = [6, 1]
        self.corresp_dict['neck_2'] = [8, 1]

        self.corresp_dict['left_knee_0'] = [21, 1]
        self.corresp_dict['right_knee_0'] = [29, 1]

        self.corresp_dict['left_shoulder_0'] = [36, -1]
        self.corresp_dict['left_shoulder_1'] = [37, 1]
        self.corresp_dict['left_shoulder_2'] = [38, -1]

        self.corresp_dict['left_elbow_0'] = [39, 1]

        self.corresp_dict['left_wrist_0'] = [42, 1]
        self.corresp_dict['left_wrist_1'] = [41, -1]
        self.corresp_dict['left_wrist_2'] = [40, -1]

        self.corresp_dict['right_shoulder_0'] = [45, -1]
        self.corresp_dict['right_shoulder_1'] = [46, -1]
        self.corresp_dict['right_shoulder_2'] = [47, 1]

        self.corresp_dict['right_elbow_0'] = [48, -1]

        self.corresp_dict['right_wirst_0'] = [51, 1]
        self.corresp_dict['right_wrist_1'] = [50, -1]
        self.corresp_dict['right_wrist_2'] = [49, -1]
