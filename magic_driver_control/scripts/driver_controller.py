#!/usr/bin/env python


"""
This node talks directly to the pololu driver
It takes in PID control effort and maps it to commands for the pololu driver
"""
from __future__ import division
import rospy

from std_msgs.msg import Float64, Int8MultiArray

DRIVE_PUB_INDEX = 0
YAW_PUB_INDEX = 1

SIDE_IR_THRESH = 10
FRONT_IR_THRESH = 21

DOORWAY_THRESH = 5
DOORWAY_IGNORES = 250

class DriverControl:
    def __init__(self):
        rospy.Subscriber("yaw/control_effort", Float64, self.yaw_ce_callback)
        rospy.Subscriber("drive/control_effort", Float64, self.drive_ce_callback)
        rospy.Subscriber("distance", Float64, self.side_dist_callback)
        rospy.Subscriber("front_distance", Float64, self.front_dist_callback)
        self.pololu_pub = rospy.Publisher('move_setpoints', Int8MultiArray, queue_size=5)
        self.drive = 0
        self.yaw = 0
        self.side_distance = 5
        self.front_distance = 100
        rospy.loginfo("Driver Controller Initialized")
        self.doorway = False
        self.doorway_count = 0
        
    def front_dist_callback(self, msg):
        self.front_distance = msg.data

    def side_dist_callback(self,msg):
        self.side_distance = msg.data
        
    def drive_ce_callback(self, msg):
        self.drive = msg.data
        # conversions... if sm we probably don't need to convert
        self.publish_cmd()
    
    def yaw_ce_callback(self, msg):
        """
        This comes in the form of around 3???
        Map it to something between -100 and 100?
        """
        self.yaw = -msg.data
        # conversions...
        self.publish_cmd()

    def publish_cmd(self):
        """
        Sends a command to the pololu
        """

        yaw = self.yaw
        if self.doorway:
            self.doorway_count -= 1
            if self.doorway_count < 0:
                rospy.logwarn("Leaving Doorway")
                self.doorway = False
            else:
                rospy.logwarn("Doorway")
                yaw = 0
        elif self.side_distance > SIDE_IR_THRESH and self.front_distance > FRONT_IR_THRESH:
            self.doorway_count += 1
            if self.doorway_count > DOORWAY_THRESH:
                self.doorway = True
                self.doorway_count = DOORWAY_IGNORES
        else:
            self.doorway_count = 0
            
        data = []
        data.append(int(self.drive))
        data.append(int(yaw))
        msg = Int8MultiArray(); msg.data = data
        self.pololu_pub.publish(msg)

def main():
    rospy.init_node("driver_controller")
    dc = DriverControl()
    rospy.spin()

if __name__ == "__main__":
    main()
