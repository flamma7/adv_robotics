#!/usr/bin/env python

import smach
import rospy
from magic_states.magic_state import Magic_State

class Corner(Magic_State):
    outcomes = ['end_corner']
    def __init__(self):
        super(Corner, self).__init__(self.outcomes)

    def execute(self, userdata):
        return "end_corner"

