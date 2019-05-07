#!/usr/bin/env python
from __future__ import division
import rospy
import sys
import signal
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from std_msgs.msg import Int8MultiArray, Int16, Empty
from pololu import PololuController, WeightedAverage

DRIVE_INDEX = 0
YAW_INDEX = 1

class PololuNode:
    ir_channel_front = 0
    ir_channel_back = 5
    drive_channel = 1
    yaw_channel = 2
    numTerms2AverageOver = 20

    def __init__(self):
        pololu = '/dev/ttyACM' + str(rospy.get_param('~pol', '0'))
        self.pololu = PololuController(self.drive_channel, self.yaw_channel, self.ir_channel_front, self.ir_channel_back, pololu)
        self.pololu.killMotors()

        self.ir_pub = rospy.Publisher('distance', Float64, queue_size=1)
        self.front_ir_pub = rospy.Publisher('front_distance', Float64, queue_size=1)
        rospy.Subscriber('move_setpoints', Int8MultiArray, self.setpoint_callback)
        rospy.Subscriber('kill_motors', Empty, self.kill_callback)
        rospy.Subscriber('revive_motors', Empty, self.revive_callback)
        rospy.Subscriber('brake_max', Empty, self.brake_callback)
        self.raw_ir_pub = rospy.Publisher('raw_ir', Int16, queue_size=10)
        self.front_unfiltered_ir_pub = rospy.Publisher('unfiltered_front_ir', Float64, queue_size=10)


        ir_read_freq = 1 / 50
        self.timer = rospy.Timer(rospy.Duration(ir_read_freq), self.ir_callback)
        rospy.loginfo("Pololu Driver Initialized.")
        self.sideWeightedAvg = WeightedAverage(self.numTerms2AverageOver)
        self.frontWeightedAvg = WeightedAverage(2*self.numTerms2AverageOver)
        self.killed = True

        signal.signal(signal.SIGINT, self.exit_graceful)
        signal.signal(signal.SIGTERM, self.exit_graceful)

    def setpoint_callback(self, msg):
        if self.killed:
            self.pololu.killMotors()
            return

        elif self.brake:
            self.pololu.brakeMotors()
            return
        
        for data in msg.data:
            if abs(data) > 100:
                rospy.logerr("Invalid Command Input: " + str(data))
                return
        self.pololu.setMotors(msg.data)

    def revive_callback(self, msg):
        self.killed = False
        self.brake = False
        rospy.logwarn("Reviving")

    def brake_callback(self,msg):
        self.brake = True
        self.killed = False
        self.pololu.brakeMotors()
        rospy.logwarn("Braking")        

    def kill_callback(self,msg):
        self.killed = True
        self.brake = False
        self.pololu.killMotors()
        rospy.logwarn("Killed")
#        rospy.signal_shutdown('Motors Killed')

        
    def validate_distance(self, d):
        max_dist = 30
        if d < 0 or d > max_dist:
            return max_dist
        else:
            return d

    def ir_callback(self,msg):
        # logistic regression
        V = self.pololu.getPosition(channel=self.ir_channel_front)
        m = 0.0008073863884277423
        b = -0.20900036625160207
        k = 0.43
        distance = (1/(m*V + b)) - k
        msg = Float64()
        msg.data = distance
        self.front_unfiltered_ir_pub.publish(msg)
        #distance = V
        # rospy.loginfo(distance)
        newFrontAvg = self.frontWeightedAvg.getNewAvg(self.validate_distance(distance))
        self.front_ir_pub.publish(newFrontAvg)
        V2=self.pololu.getPosition(channel=self.ir_channel_back)
#        rospy.loginfo(V2)
        msg = Int16()
        msg.data = int(V2)
        self.raw_ir_pub.publish(msg)
        distance2 = (1/(m*V2+b)) - k
        newAvgDist = self.sideWeightedAvg.getNewAvg(self.validate_distance(distance2))
        rospy.loginfo(newAvgDist)
        # rospy.loginfo(newAvgDist)
        self.ir_pub.publish(newAvgDist)

    def exit_graceful(self, signum, frame):
        self.pololu.killMotors()
        self.killed = True
        rospy.logerr("Graceful Kill")
        sys.exit(0)

def main():
    rospy.init_node('pololu_node')
    pol = PololuNode()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
