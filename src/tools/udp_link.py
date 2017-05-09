#!/usr/bin/env python
import socket
import struct
import rospy


class UDPLink(object):
    def __init__(self, ip="BAXTERFLOWERS.local", port=5005):
        self.ip = ip
        self.port = port

    def _send_data(self, channel, data, string_pattern):
        str_pat = 'I'
        if string_pattern != 's':
            str_pat += string_pattern
            packer = struct.Struct(str_pat)
            sent_vect = [channel] + data
            packed_data = packer.pack(*sent_vect)
        else:
            packer = struct.Struct(str_pat)
            sent_vect = [channel]
            packed_data = packer.pack(*sent_vect)
            packed_data += data
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        try:
            sock.sendto(packed_data, (self.ip, self.port))
        except socket.gaierror:
            rospy.logwarn("Host not connected")

    def send_string(self, channel, string_value):
        self._send_data(channel, string_value, 's')

    def send_string_vector(self, channel, vect):
        # concatenate the list in one string
        string_value = ""
        if vect:
            for s in vect:
                string_value += (s + "|")
            # remove last delimiter
            string_value = string_value[:-1]
        else:
            string_value = "none"
        self.send_string(channel, string_value)

    def send_int(self, channel, int_value):
        self._send_data(channel, int_value, 'I')

    def send_float_vector(self, channel, vect):
        self._send_data(channel, vect, ('f' * len(vect)))
