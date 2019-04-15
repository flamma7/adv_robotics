#!/usr/bin/env python

import smach
import rospy
import tf
import math
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from magic_states.magic_state import Magic_State

class Doorway(Magic_State):
    outcomes = ['end_doorway']
    yaw_setpoint = 0
    CONST_DRIVE = 10
    
    def __init__(self, update_rate,drive):
        super(Doorway, self).__init__(update_rate, self.outcomes)
        rospy.Subscriber('/imu/data/', Imu, self.imu_data_callback)
        rospy.Subscriber('door/control_effort', Float64, self.door_ce_callback)
        self.imu_state_pub = rospy.Publisher('door/state', Float64, queue_size=10)
        self.imu_setpoint_pub = rospy.Publisher('door/setpoint', Float64, queue_size=10)
        #self.yawPub = rospy.Publisher('/imu/yaw', Float64, queue_size=1)
        
        #self.doorCtl = rospy.Publisher('')
        self.drive = drive

    def imu_data_callback(self, msg):
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        euler_orient = tf.transformations.euler_from_quaternion(quat)
        self.yawPub.publish(euler_orient[2]) #only care about yaw data
        self.yaw = euler_orient[2]
        state = self.correct_angle(self.yaw_setpoint, self.yaw)
        msg = Float64()
        msg.data = state
        imu_state_pub.publish(msg)
        

    def door_ce_callback(self, msg):
        #msg.data should be a yaw value between -100 and 100
        self.yaw_ce = msg.data
        
        
    def execute(self, userdata):
        rate = rospy.Rate(self.update_rate)
        self.yaw_setpoint = self.yaw
        sp_msg = Float64()
        sp_smg.data = self.yaw_setpoint
        while not rospy.is_shutdown():
            if not self.doorway_check():
                return 'end_doorway'
            self.imu_setpoint_pub(sp_msg)
            self.publish_cmd(self.drive, self.yaw_ce)
            rate.sleep()
        return "end_doorway"

    def correct_angle(self, yaw_sp, yaw_state):
        new_angle = yaw_sp - yaw_state
        if new_angle < -math.pi:
            new_angle += 2*math.pi
        elif new_angle > math.pi:
            new_angle -= 2*math.pi
        return new_angle

