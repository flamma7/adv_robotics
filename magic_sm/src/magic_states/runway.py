#!/usr/bin/env python

import smach
import rospy
from magic_states.magic_state import Magic_State
from sensor_msgs.msg import Imu

class Runway(Magic_State):
    outcomes = ['airborne']
    def __init__(self, update_rate, distance, jump_angle):
        super(Runway, self).__init__(update_rate, self.outcomes)
        self.airborne = False        
        self.drive = self.calculate_drive(distance, jump_angle)
        self.yaw = 0 # Determine if PID is needed
        rospy.Subscriber('/imu/data/', Imu, self.imu_callback)
        self.imu_thresh = float(rospy.get_param('imu_thresh'))
        self.accel_z = None
        num_terms_to_avg = 50
        self.beta = (1.0 - float(num_terms_to_avg)) / (- float(num_terms_to_avg))

    def calculate_drive(self, distance, jump_angle):
        # TODO actually calculate the required drive
        return int(rospy.get_param('drive'))

    def execute(self, userdata):
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown() and not self.airborne:
            self.publish_cmd(self.drive, self.yaw)
            rate.sleep()
        return "airborne"

    def imu_callback(self, msg):
        if self.accel_z == None:
            self.accel_z = msg.linear_acceleration.z
        else:
            self.accel_z = (self.beta * self.accel_z) + (1-self.beta) * msg.linear_acceleration.z
            
        if abs(msg.linear_acceleration.z) < self.imu_thresh:
            self.airborne = True
