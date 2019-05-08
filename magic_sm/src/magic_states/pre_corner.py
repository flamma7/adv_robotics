#!/usr/bin/env pybthon

import smach
import rospy
from magic_states.magic_state import Magic_State

class Pre_Corner(Magic_State):
    outcomes = ['end_pre_corner']
    def __init__(self, update_rate, drive):
        super(Corner, self).__init__(update_rate, self.outcomes)

    def execute(self, userdata):
        rate = rospy.Rate(self.update_rate)
        breaking = self.break_gen(10)
        while not rospy.is_shutdown() and self.front_wall() and breaking.next():
            self.publish_cmd(-100, 0)
            rate.sleep()
        return 'end_pre_corner'

    def break_gen(self, n):
        for i in range(n+1):
            if i == n:
                yield False
            else:
                yield True
