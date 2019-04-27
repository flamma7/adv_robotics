#!/usr/bin/env python

import smach
import rospy
from magic_states.magic_state import Magic_State

class Corner(Magic_State):
    outcomes = ['end_corner']
    def __init__(self, update_rate, drive):
        super(Corner, self).__init__(update_rate, self.outcomes)
        self.drive = drive
        self.rate = update_rate

    def execute(self, userdata):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown() and self.corner_check():
            self.publish_cmd(self.drive, 100)
            rate.sleep()
        return "end_corner"

