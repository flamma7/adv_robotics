#!/usr/bin/env python

import smach
import rospy
import tf
from magic_states.magic_state import Magic_State
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Empty
import math

class Runway(Magic_State):
    outcomes = ['airborne']
    def __init__(self, update_rate, distance, jump_angle):
        super(Runway, self).__init__(update_rate, self.outcomes)
        self.airborne = False        
        self.drive = self.calculate_drive(distance, jump_angle)
        self.yaw = None
        self.yaw_ce = 0
        self.yaw_setpoint = None
        self.state_pub = rospy.Publisher('/imu_pid/state', Float64, queue_size=1)
        self.setpoint_pub = rospy.Publisher('/imu_pid/setpoint',Float64, queue_size=1)
        self.imu_thresh = float(rospy.get_param('imu_thresh'))
        self.accel_z = None
        num_terms_to_avg = 20
        self.beta = (1.0 - float(num_terms_to_avg)) / (- float(num_terms_to_avg))
        self.pub = rospy.Publisher("running_avg", Float64, queue_size=10)
        self.pubz = rospy.Publisher("raw_z", Float64, queue_size=10)
        rospy.Subscriber("revive_motors",Empty, self.revive_callback)
        rospy.Subscriber('/imu/data/', Imu, self.imu_callback)
        rospy.Subscriber('/imu_pid/control_effort', Float64, self.runway_ce_callback)
        rospy.wait_for_message('/imu/data', Imu)
        
        self.revive = False
        rampup_time = float(rospy.get_param('rampup_time'))
        self.rampup_increment = self.drive / (rampup_time * self.update_rate)

    def calculate_drive(self, distance, jump_angle):
        # TODO actually calculate the required drive
        return int(rospy.get_param('drive'))

    def execute(self, userdata):
        rate = rospy.Rate(self.update_rate)
        while (not self.revive  or self.yaw is None) and not rospy.is_shutdown():
            rate.sleep()
        self.yaw_setpoint = self.yaw
        sp = Float64(); sp.data=0.0
        self.setpoint_pub.publish(sp)                    
        drive = 10
        while not rospy.is_shutdown() and not self.airborne:
            self.setpoint_pub.publish(sp)
            self.publish_cmd(drive, self.yaw_ce)
            if drive < self.drive:
                drive += self.rampup_increment
            elif drive > self.drive:
                drive = self.drive
            rate.sleep()
        return "airborne"

    def imu_callback(self, msg):
        quat_list = [msg.orientation.x, msg.orientation.y,msg.orientation.z,msg.orientation.w]
        eul = tf.transformations.euler_from_quaternion(quat_list)
        fyaw = Float64()
        if self.yaw_setpoint is not None:
            self.yaw = self.correct_angle(self.yaw_setpoint, eul[2])
            fyaw.data = self.yaw
            self.state_pub.publish(fyaw)
        else:
            self.yaw = eul[2]

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


    def revive_callback(self,msg):
        self.revive = True

    def runway_ce_callback(self,msg):
        self.yaw_ce = - msg.data

    
    def correct_angle(self, yaw_sp, yaw_state):
        new_angle = yaw_state - yaw_sp
        if new_angle < -math.pi:
            new_angle += 2*math.pi
        elif new_angle > math.pi:
            new_angle -= 2*math.pi
        return new_angle