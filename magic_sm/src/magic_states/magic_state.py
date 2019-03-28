#!/usr/bin/env python

import smach
import rospy
from std_msgs.msg import Bool, Int8MultiArray

class Magic_State(smach.State):

    def __init__(self, update_rate, outcomes):
        rospy.Subscriber('doorway', Bool, self.doorway_callback)
        rospy.Subscriber('corner', Bool, self.corner_callback)
        self.pololu_pub = rospy.Publisher('move_setpoints', Int8MultiArray, queue_size=5)
        self.update_rate = update_rate
        self._corner = 0
        self._doorway = 0

        super(Magic_State, self).__init__(outcomes=outcomes)

    def doorway_callback(self, msg):
        self._doorway = msg.data

    def corner_callback(self, msg):
        self._corner = msg.data

    def corner_check(self):
        return self._corner
    
    def doorway_check(self):
        return self._doorway

    def publish_cmd(self, drive, yaw):
        data = []
        data.append(int(drive))
        data.append(int(yaw))
        msg = Int8MultiArray(); msg.data = data
        self.pololu_pub.publish(msg)
