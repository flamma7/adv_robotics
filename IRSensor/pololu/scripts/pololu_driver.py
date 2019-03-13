#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from std_msgs.msg import Int8MultiArray
from pololu import PololuController, WeightedAverage

DRIVE_INDEX = 0
YAW_INDEX = 1

class PololuNode:
    ir_channel0 = 0
    ir_channel1 = 5
    drive_channel = 1
    yaw_channel = 2
    numTerms2AverageOver = 10

    def __init__(self):

        self.pololu = PololuController(self.drive_channel, self.yaw_channel, self.ir_channel0, self.ir_channel1)
        
        self.ir_pub = rospy.Publisher('distance', Float64, queue_size=10)
        rospy.Subscriber('setpoints', Int8MultiArray, self.setpoint_callback)
        rospy.Subscriber('kill_motors', Empty, self.kill_callback)

        self.timer = rospy.Timer(rospy.Duration(2), self.ir_callback)
        rospy.loginfo("Pololu Driver Initialized.")
        self.weightedAvg = WeightedAverage(self.numTerms2AverageOver)

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

        V = self.pololu.getPosition(channel=self.ir_channel0)
        m = 0.0008073863884277423
        b = -0.20900036625160207
        k = 0.43
        distance = (1/(m*V + b)) - k
        #distance = V
        rospy.loginfo(distance)
        self.ir_pub.publish(distance)
        V2=self.pololu.getPosition(channel=self.ir_channel1)
        distance2 = (1/(m*V2+b)) - k
        newAvgDist = self.weightedAvg.getNewAvg(distance2)
        rospy.loginfo(newAvgDist)
        self.ir_pub.publish(distance2)


def main():
    rospy.init_node('pololu_node')
    pol = PololuNode()

    rospy.spin()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
