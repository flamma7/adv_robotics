#!/usr/bin/env python


from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
from std_msgs.msg import Int8MultiArray
import rospy

# Publishers
DRIVE_PUB_INDEX = 0
YAW_PUB_INDEX = 1

# ADCs
DRIVE_INDEX = 1
YAW_INDEX = 2

# Buttons
ACTIVE_INDEX = 0
SKID_INDEX = 1
REVIVE_INDEX = 2
STRAIGHT_INDEX = 3
KILL_INDEX_R1 = 5
UNARMED_INDEX = 6
ARMED_INDEX = 7

class DualShock():

    def __init__(self):
        rospy.Subscriber('joy', Joy, self.callback)
        self.kill_pub = rospy.Publisher('kill_motors', Empty, queue_size=1)
        self.skid_kill_pub = rospy.Publisher('/is_kill', Empty, queue_size=1)
        self.skid_straight_pub = rospy.Publisher('/is_straight',Empty,queue_size=1)
        self.skid_skid_pub = rospy.Publisher('/is_skid',Empty, queue_size=1)
        self.revive_pub = rospy.Publisher('/revive_motors', Empty, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_max', Empty, queue_size=1)
        self.setpoint_pub = rospy.Publisher('move_setpoints', Int8MultiArray, queue_size=5)
        self.motor_control = rospy.get_param("~motor_control", False)
        self.armed = False
        rospy.logwarn("Initializing unarmed")
        
    def callback(self, msg):
        buttons = msg.buttons
        if buttons[KILL_INDEX_R1]:
            rospy.logwarn("Killed")
            self.kill_pub.publish(Empty())
        elif buttons[SKID_INDEX]:
            rospy.logwarn("Skidding")
            self.skid_skid_pub.publish(Empty())
        elif buttons[STRAIGHT_INDEX]:
            rospy.logwarn("Straighting")
            self.skid_straight_pub.publish(Empty())
        elif buttons[REVIVE_INDEX]:
            rospy.logwarn("Reviving")
            self.revive_pub.publish(Empty())
        elif buttons[ACTIVE_INDEX]:
            rospy.logwarn("Braking")
            self.brake_pub.publish(Empty())
        elif buttons[ARMED_INDEX]:
            rospy.logwarn("Armed")
            self.armed = True
        elif buttons[UNARMED_INDEX]:
            rospy.logwarn("Unarmed")
            self.armed = False
        elif self.armed:
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
