#!/usr/bin/env python

import smach
import rospy
from magic_states.magic_state import Magic_State

class Straight(Magic_State):
    outcomes = ['completed', 'doorway', 'corner']
    def __init__(self):
        super(Straight, self).__init__(self.outcomes)
        self.count = 0

    def execute(self, userdata):
        self.count += 1
        if self.count == 1:
            return "doorway"
        elif self.count == 2:
            return "corner"
        else:
            return "completed"
