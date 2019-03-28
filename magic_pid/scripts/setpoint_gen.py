#!/usr/bin/env python
"""
Publishes the setpoint at a regular rate
Default 3 feet
"""
import rospy
from std_msgs.msg import Float64

def main():
    rospy.init_node("setpoint_generator")
    rospy.loginfo("Setpoint Generator Initialized")
    pub = rospy.Publisher("setpoint", Float64, queue_size=1)
    rate = rospy.Rate(30)

    msg = Float64()
    msg.data = 5.0

    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    main()
