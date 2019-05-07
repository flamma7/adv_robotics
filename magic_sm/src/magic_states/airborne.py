#!/usr/bin/env python

import smach
import rospy
import tf
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from magic_states.magic_state import Magic_State

class Airborne(Magic_State):
    outcomes = ['landing']
    
    def __init__(self, update_rate, jump_angle):
        super(Airborne, self).__init__(update_rate, self.outcomes)
        self.landing = False        
        rospy.Subscriber('/running_avg', Float64, self.avg_callback)
        self.imu_thresh = float(rospy.get_param('imu_thresh'))

        self.air_drive = int(rospy.get_param('airborne_drive'))
    def avg_callback(self, msg):
        if abs(msg.data) > self.imu_thresh:
            self.landing = True
        
    def execute(self, userdata):
        rospy.loginfo("airborne")
        self.landing = False
        rate = rospy.Rate(self.update_rate)
        back_rot = self.air_drive
        self.publish_cmd(back_rot,0)
#        self.publish_cmd(0,0)
        while not rospy.is_shutdown() and not self.landing:
            self.publish_cmd(back_rot,0)
#            self.publish_cmd(0,0)
            rate.sleep()
        return "landing"

