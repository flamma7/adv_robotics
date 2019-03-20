#!/usr/bin/env python
from __future__ import division
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from std_msgs.msg import Int8MultiArray, Int16
from pololu import PololuController, WeightedAverage

DRIVE_INDEX = 0
YAW_INDEX = 1

class PololuNode:
    ir_channel_front = 0
    ir_channel_back = 5
    drive_channel = 1
    yaw_channel = 2
    numTerms2AverageOver = 10

    def __init__(self):

        self.pololu = PololuController(self.drive_channel, self.yaw_channel, self.ir_channel_front, self.ir_channel_back)

        self.ir_pub = rospy.Publisher('distance', Float64, queue_size=10)
        self.front_ir_pub = rospy.Publisher('front_distance', Float64, queue_size=10)
        rospy.Subscriber('move_setpoints', Int8MultiArray, self.setpoint_callback)
        rospy.Subscriber('kill_motors', Empty, self.kill_callback)
        self.raw_ir_pub = rospy.Publisher('raw_ir', Int16, queue_size=10)

        ir_read_freq = 1 / 50
        self.timer = rospy.Timer(rospy.Duration(ir_read_freq), self.ir_callback)
        rospy.loginfo("Pololu Driver Initialized.")
        self.weightedAvg = WeightedAverage(self.numTerms2AverageOver)
        self.killed = False

    def setpoint_callback(self, msg):
        if self.killed:
            return
        
        for data in msg.data:
            if abs(data) > 100:
                rospy.logerr("Invalid Command Input: " + str(data))
                return
        self.pololu.setMotors(msg.data)


    def kill_callback(self,msg):
        self.killed = True
        self.pololu.killMotors()
        rospy.signal_shutdown('Motors Killed')

    def validate_distance(d):
        if d < 0 or d > 18:
            return 18
        else:
            return d

    def ir_callback(self,msg):
        # logistic regression
        V = self.pololu.getPosition(channel=self.ir_channel_front)
        m = 0.0008073863884277423
        b = -0.20900036625160207
        k = 0.43
        distance = (1/(m*V + b)) - k
        #distance = V
        # rospy.loginfo(distance)
        self.front_ir_pub.publish(validate_distance(distance))
        V2=self.pololu.getPosition(channel=self.ir_channel_back)
        msg = Int16
        msg.data = V2
        self.raw_ir_pub.publish(msg)
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
