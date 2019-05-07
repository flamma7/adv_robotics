#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

class StateMachine:

    def __init__(self):
        rospy.loginfo("State Machine Initializing")
        self.drivePub = rospy.Publisher("drive/control_effort", Float64, queue_size=10)
        self.drive_effort = 8
        self.timer = rospy.Timer(rospy.Duration(0.1), self.setpoint_callback)

    def dist_callback(self, msg):
        self.distance = msg.data

    def setpoint_callback(self, msg):
        new_msg = Float64()
        new_msg.data = self.drive_effort
        self.drivePub.publish(new_msg)

def main():
    rospy.init_node('state_machine')
    sm = StateMachine()
    rospy.spin()


if __name__ == "__main__":
    main()
