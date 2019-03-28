#!/usr/bin/env python

import smach
import rospy
from std_msgs.msg import Bool

class Magic_State(smach.State):

    def __init__(self, outcomes):
        rospy.Subscriber('doorway', Bool, self.doorway_callback)
        rospy.Subscriber('corner', Bool, self.corner_callback)

        super(Magic_State, self).__init__(outcomes=outcomes)

    def doorway_callback(self, msg):
        self._doorway = msg.data

    def corner_callback(self, msg):
        self._corner = msg.data

    def corner_check(self):
        return self._corner
    
    def doorway_check(self):
        return self._doorway
