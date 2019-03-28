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

    CONST_DRIVE = 10
    
    def __init__(self, update_rate):
        super(Doorway, self).__init__(update_rate, self.outcomes)
        rospy.Subscriber('/imu/data/', Imu, self.imu_data_callback)
        rospy.Subscriber('doorway_control_effort', Float64, self.doorway_pid_callback)
        self.yawPub = rospy.Publisher('/imu/yaw', Float64, queue_size=1)
        self.pidYaw = rospy.Publisher('/pid/yaw', Float64, queue_size=1)
        #self.doorCtl = rospy.Publisher('')

    def imu_data_callback(self, msg):
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        euler_orient = tf.transformations.euler_from_quaternion(quat)
        self.yawPub.publish(euler_orient[2]) #only care about yaw data
        yaw_for_pid = euler_orient[2] / math.pi * 180
        self.pidYaw.publish(yaw_for_pid)

    def doorway_pid_callback(self, msg):
        #msg.data should be a yaw value between -100 and 100
        self.publish_cmd(CONST_DRIVE, msg.data)
        
        
    def execute(self, userdata):
        rospy.sleep(10)
        return "end_doorway"

