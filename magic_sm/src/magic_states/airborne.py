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
        rospy.Subscriber('/imu/data/', Imu, self.imu_callback)
        self.imu_thresh = float(rospy.get_param('imu_thresh'))        
        # Initialize PID control

    def imu_callback(self, msg):
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quat)
        # publish state to the PID loop
        if abs(msg.linear_acceleration.z) > self.imu_thresh:
            self.landing = True
        
    def execute(self, userdata):
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown() and not self.landing:
            self.publish_cmd(0,0)
        return "landing"

