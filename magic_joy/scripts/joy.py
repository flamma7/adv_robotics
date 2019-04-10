#!/usr/bin/env python


from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
from std_msgs.msg import Int8MultiArray
import rospy

DRIVE_PUB_INDEX = 0
YAW_PUB_INDEX = 1

KILL_INDEX_R1 = 5
DRIVE_INDEX = 1
YAW_INDEX = 3

SKID_INDEX = 6      #FIXME
STRAIGHT_INDEX = 7  #FIXME

class DualShock():

    def __init__(self):
        rospy.Subscriber('joy', Joy, self.callback)
        self.kill_pub = rospy.Publisher('kill_motors', Empty, queue_size=1)
        self.skid_kill_pub = rospy.Publisher('/is_kill', Empty, queue_size=1)
        self.skid_straight_pub = rospy.Publisher('/is_straight',Empty,queue_size=1)
        self.skid_skid_pub = rospy.Publisher('/is_skid',Empty, queue_size=1)
        self.setpoint_pub = rospy.Publisher('move_setpoints', Int8MultiArray, queue_size=5)
        self.motor_control = True

    def callback(self, msg):
        buttons = msg.buttons
        if buttons[KILL_INDEX_R1]:
            rospy.logwarn("Killed")
            self.kill_pub.publish(Empty())
        elif buttons[SKID_INDEX]:
            rospy.log("Skidding")
            self.skid_skid_pub.publish(Empty())
        elif buttons[STRAIGHT_INDEX]:
            rospy.log("Straighting")
            self.skid_straight_pub.publish(Empty())
    
        elif self.motor_control:
            raw_drive = msg.axes[DRIVE_INDEX]
            drive = self.transform_drive(raw_drive)
            
            raw_yaw =  msg.axes[YAW_INDEX]
            yaw = self.transform_yaw(raw_yaw)
            data = [];
            data.append(drive)
            data.append(yaw)
            msg = Int8MultiArray(); msg.data = data
            self.setpoint_pub.publish(msg)
            
            if drive:
                rospy.loginfo("drive: " + str(drive))
            if yaw:
                rospy.loginfo("yaw: " + str(yaw))

    def transform_yaw(self, raw_yaw):
        """
        Pololu driver expects yaw -100, 100
        """
        return -int(raw_yaw * 100)

    def transform_drive(self, raw_drive):
        """
        Pololu driver expects drive in range 0-100
        """
        return int(raw_drive * 100)
        # if raw_drive < 0:
        #     return 0
        # else:
        #     return int(raw_drive * 100)
        

def main():
    rospy.init_node('dual_shock')
    d = DualShock()
    rospy.spin()

if __name__ == "__main__":
    main()
