#!/usr/bin/env python

import smach
import rospy
from magic_states.magic_state import Magic_State

class Doorway(Magic_State):
    outcomes = ['end_doorway']
    
    def __init__(self):
        super(Doorway, self).__init__(self.outcomes)

    def execute(self, userdata):
        return "end_doorway"

