# Dependencies
import math
import tf
import numpy as np
import matplotlib.pyplot as plt
import os.path
import scipy.stats
# ROS dependencies
import rospy
import geometry_msgs.msg as geo_msgs
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
        self.vel_pub = rospy.Publisher("/velocity", std_msgs.Float64)

        self.ir_freq = 1/50
        self.timer = rospy.Timer(rospy.Duration(self.ir_freq), self.ir_freq_callback)

        self.vel_freq = 1/10
        self.vel_pub_timer = rospy.Timer(rospy.Duration(self.vel_freq), self.vel_callback)

        self.vels = []


    def getFrontDistance(self, data):
        self.prev_dist = self.cur_dist
        self.cur_dist = data

    def ir_freq_callback(self):
        pass

    def vel_callback(self):
        if len(self.vels) < 5:
            return
        self.vel_pub.publish(np.mean(self.vels[-5:]))

if __name__=="__main__":
    try:
		main()
	except rospy.ROSInterruptException:
		pass
