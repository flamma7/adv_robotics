#!/usr/bin/env python

import smach
import rospy
from magic_states.magic_state import Magic_State
from std_msgs.msg import Float64

class Straight(Magic_State):
    outcomes = ['completed', 'doorway', 'corner']
    def __init__(self, update_rate):
        super(Straight, self).__init__(update_rate, self.outcomes)
        self.drive = rospy.get_param('straight/drive')
        rospy.Subscriber('yaw/control_effort', Float64, self.yaw_control_effort_callback)
        self.yaw_ce = 0

    def execute(self, userdata):
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            # Check for recent updates
            if self.corner_check():
                return "corner"
            elif self.doorway_check():
                return "doorway"
            self.publish_cmd(self.drive, self.yaw_ce)
            rate.sleep()
        return "completed"

    def yaw_control_effort_callback(self, msg):
        self.yaw_ce = msg.data



            
