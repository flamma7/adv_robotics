# Dependencies
import math
import tf
import numpy as np
import matplotlib.pyplot as plt
import os.path
# ROS dependencies
import rospy
import std_msgs.msg as std_msgs

def main():
    mj = MagicJump()
    rospy.spin()

class MagicJump():
    def __init__(self):
        # Setup node
        rospy.init_node('magic_jump')

        # Retrieve IR distance
        rospy.Subscriber("/front_distance", std_msgs.Float64, self.getFrontDistance)
        self.cur_dist = 0
        self.prev_dist = 0
        self.vel_pub = rospy.Publisher("/velocity", std_msgs.Float64, queue_size=10)

        self.vel_freq = 1/10
        self.N_terms = 5
        self.vels = []
        self.vel_pub_timer = rospy.Timer(rospy.Duration(self.vel_freq), self.vel_callback)


    def getFrontDistance(self, data):
        self.prev_dist = self.cur_dist
        self.cur_dist = data
        ir_freq = 1/50

        self.vel.append(abs((self.prev_dist - self.cur_dist)/ ir_freq))

    def vel_callback(self):
        if len(self.vels) < self.N_terms:
            return
        self.vel_pub.publish(weighted_avg(np.mean(self.vels[-self.N_terms:-1]), self.vels[-1]))

    def weighted_avg(self, avg, new):
        beta = 1 - (1/self.N_terms)
        return beta * avg + (1-beta) * newInput


if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass