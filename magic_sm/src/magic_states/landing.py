#!/usr/bin/env python

import smach
import rospy
from magic_states.magic_state import Magic_State
from std_msgs.msg import Float64

class Landing(Magic_State):
    outcomes = ['completed']
    def __init__(self, update_rate, landing_speed, landing_time):
        super(Landing, self).__init__(update_rate, self.outcomes)
        self.drive = landing_speed
        self.landing_time = landing_time
        # Hopefully we don't need to PID here also!
        
    def execute(self, userdata):
        rospy.loginfo("landing")
        # self.publish_cmd(self.drive, 0)
        self.publish_cmd(self.drive, 0) # Stop immediately
        rospy.sleep(self.landing_time)
        self.publish_cmd(-100, 0) # Full stop
        return "completed"


            
