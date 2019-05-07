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
	robberEvasion(copName=copName.lower(), robberName=robName.lower())
	rospy.spin()

class robberEvasion():
	def __init__(self, copName, robberName):
		# Setup node
		rospy.init_node('magic_jump')

		# Retrieve IR distance
		rospy.Subscriber("/front_distance", std_msgs.Float64, self.getFrontDistance)
		self.front_dist = 0
        self.vel_pub = rospy.Publisher("/velocity", std_msgs.Float64)

        self.freq = 1/50
        self.timer = rospy.Timer(rospy.Duration(self.freq), self.freq_callback)


    def getFrontDistance(self, data):
        self.front_dist = data

    def freq_callback(self):
        pass

if __name__=="__main__":
    main()
