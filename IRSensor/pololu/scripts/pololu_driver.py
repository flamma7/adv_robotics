#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from std_msgs.msg import Float64MultiArray
from pololu import PololuController

DRIVE_INDEX = 0
YAW_INDEX = 1

class PololuNode:
    ir_channel0 = 0
    ir_channel1 = 1
    drive_channel = 2
    yaw_channel = 3

    def __init__(self, channel_map):
        
        self.pololu = PololuController(self.drive_channel, self.turn_channel, self.ir_channel0, self.ir_channel1)
        
        self.ir_pub = rospy.Publisher('distance', Float64, queue_size=10)
        rospy.Subscriber('setpoints', Float64MultiArray, self.setpoint_callback)
        rospy.Subscriber('kill_motors', Empty, self.kill_callback)

        self.timer = rospy.Timer(rospy.Duration(1/20), self.ir_callback)

    def setpoint_callback(self, msg):
        for data in msg.data:
            if abs(data) > 100:
                rospy.logerr("Invalid Command Input: " + str(data))
                return
        self.pololu.setMotors(msg.data)


    def kill_callback(self,msg):
        self.pololu.killMotors(msg, drive_channel, yaw_channel)
        rospy.signal_shutdown('Motors Killed')


    def ir_callback(self,msg):

        V = self.pololu..getPosition(channel=ir_channel)
        m = 0.0008073863884277423
        b = -0.20900036625160207
        k = 0.43
        distance = (m/(V + b)) - k
        rospy.loginfo(distance)
        self.ir_pub.publish(distance)

                    
        

# def sharp_ir_sensor():
#     pub = rospy.Publisher('distance', Float64, queue_size=10)
#     rospy.init_node('sharp_ir_sensor', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     maestro = PololuController()
#     while not rospy.is_shutdown():
#         V = maestro.getPosition(channel=args.channel)
#         m = 0.0008073863884277423
#         b = -0.20900036625160207
#         k = 0.43
#         distance = (m/(V + b)) - k
#         rospy.loginfo(distance)
#         pub.publish(distance)
#         rate.sleep()
# def set_drive():
#     maestro = PololuController()
#     rospy.Subscriber('drive_setpoints', maestro.setDrive())

#     rospy.spin()

# def kill_motors():
#     maestro = PololuController()
#     rospy.Subscriber('kill_motors', maestro.killMotors())

#     rospy.spin()

def main():
    rospy.init_node('pololu_node')
    pol = PololuNode()

    rospy.spin()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
