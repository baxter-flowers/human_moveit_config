#!/usr/bin/env python
import socket
import struct


class UDPLink(object):
    def __init__(self, ip="BAXTERFLOWERS.local", port=5005):
        self.ip = ip
        self.port = port

    def _send_data(self, data):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        sock.sendto(data, (self.ip, self.port))

    def send_int(self, channel, int_value):
        # send channel
        self._send_data(channel)
        self._send_data(struct.pack('I', int_value))

    def send_float_vector(self, channel, vect):
        # send channel
        self._send_data(channel)
        # create string pattern
        string_pattern = ('f ' * len(vect))[:-1]
        packer = struct.Struct(string_pattern)
        packed_data = packer.pack(*vect)
        # send the array
        self._send_data(packed_data)
