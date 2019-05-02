#!/usr/bin/env python

import tf
from math import pi
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
import imu_filter_madgwick

class YawRepub():
    yaw = 0
    def __init__(self):
        rospy.Subscriber('/imu/data', Imu, self.imu_data_callback)
        self.yawRepub = rospy.Publisher('yaw_repub', Float64, queue_size=2)
        self.yawRepubDeg = rospy.Publisher('deg_yaw_repub', Float64, queue_size=2)
  

    def imu_data_callback(self,msg):
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        eul = tf.transformations.euler_from_quaternion(quat)
        self.yaw = eul[2]
        tempmsg = Float64()
        tempmsg.data = self.yaw
        self.yawRepub.publish(tempmsg)
        tempmsg.data = self.yaw * 180 / pi
        self.yawRepubDeg.publish(tempmsg)


def main():

    rospy.init_node('yaw_repub')
    yawer = YawRepub()
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main()
    

