#!/usr/bin/env python

import rospy
from us_digital_encoders.msg import USDigitalEncoders
from driver import UsDigitalSei

class Encoders(object):

    def __init__(self):
        rospy.init_node('us_digital_encoders')
        self.max_count = 65535
        # initializations
        self.prev_data = None
        self.prev_left_ticks = None # left ticks previous
        self.prev_right_ticks = None # right ticks previous
        # subscribe to serial port, set up publisher
        self.encoders = UsDigitalSei(port='/dev/ttyUSB0', platform='TR')
        self.pub = rospy.Publisher('encoders', USDigitalEncoders, queue_size=5)

    def _generate_encoder_msg(self, delta_left_ticks, delta_right_ticks):
        msg = USDigitalEncoders()
        msg.ticks = [delta_left_ticks, delta_right_ticks]
        msg.header.stamp = rospy.Time.now()
        return msg

    def _handle_rollover(self, num):
        if num < -30000: # roll from 65535 -> 0
            num += self.max_count + 1
        elif num > 30000: # roll from 0 -> 65535
            num -= self.max_count - 1
        return num

    def _parse_data(self, data):
        left, right = data.split(',')
        left_roll, left_ticks = left.split('-')
        right_roll, right_ticks = right.split('-')
        return int(left_ticks), int(right_ticks)

    def run(self):
        while not rospy.is_shutdown():
            data = self.encoders.read()
            # store first data
            if self.prev_data is None:
                self.prev_data = data
                self.prev_left_ticks, self.prev_right_ticks = self._parse_data(data)
                continue
            # get the current ticks of each encoder
            try:
                left_ticks, right_ticks = self._parse_data(data)
            except ValueError:
                continue
            # calculate the number of ticks since the encoders were last polled and handle rollover
            delta_left_ticks = self._handle_rollover(left_ticks - self.prev_left_ticks)
            delta_right_ticks = self._handle_rollover(right_ticks - self.prev_right_ticks)
            # publish encoder data
            encoder_msg = self._generate_encoder_msg(delta_left_ticks, delta_right_ticks)
            self.pub.publish(encoder_msg)
            # record previous drive encoder data
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks

if __name__ == '__main__':
    enc = Encoders()
    enc.run()
