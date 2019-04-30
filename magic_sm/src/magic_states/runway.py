#!/usr/bin/env python

import smach
import rospy
from magic_states.magic_state import Magic_State
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

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
        num_terms_to_avg = 20
        self.beta = (1.0 - float(num_terms_to_avg)) / (- float(num_terms_to_avg))
        self.pub = rospy.Publisher("running_avg", Float64, queue_size=10)
        self.pubz = rospy.Publisher("raw_z", Float64, queue_size=10)

        rampup_time = float(rospy.get_param('rampup_time'))
        self.rampup_increment = rampup_time / self.drive

    def calculate_drive(self, distance, jump_angle):
        # TODO actually calculate the required drive
        return int(rospy.get_param('drive'))

    def execute(self, userdata):
        rate = rospy.Rate(self.update_rate)
        drive = self.rampup_increment
        while not rospy.is_shutdown() and not self.airborne:
            self.publish_cmd(drive, self.yaw)
            if drive < self.drive:
                drive += self.rampup_increment
            rate.sleep()
        return "airborne"

    def imu_callback(self, msg):
        if self.accel_z == None:
            self.accel_z = msg.linear_acceleration.z
        else:
            self.accel_z = (self.beta * self.accel_z) + (1-self.beta) * msg.linear_acceleration.z
        f = Float64(); f.data = self.accel_z
        self.pub.publish(f)
        f2 = Float64(); f2.data = msg.linear_acceleration.z
        self.pubz.publish(f2)
            
        if abs(self.accel_z) < self.imu_thresh:
            self.airborne = True
