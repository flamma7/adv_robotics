#!/usr/bin/env python
from __future__ import division
import smach
import rospy
from magic_states.magic_state import Magic_State
from std_msgs.msg import Float64, Empty

class Straight(Magic_State):
    outcomes = ['completed', 'doorway', 'corner']
    def __init__(self, update_rate,drive,wall_setpoint):
        super(Straight, self).__init__(update_rate, self.outcomes)
        self.drive = drive
        self.ir_setpt_msg = Float64()
        self.ir_setpt_msg.data = wall_setpoint
        self.ir_pid_pub = rospy.Publisher('hall/setpoint', Float64, queue_size=10)
        rospy.Subscriber('hall/control_effort', Float64, self.yaw_control_effort_callback)
        self.yaw_ce = 0
        rospy.Subscriber("/revive_motors", Empty, self.rev_callback)
        self.revived = False

    def rev_callback(self, msg):
        self.revived = True

    def execute(self, userdata):
        rate = rospy.Rate(self.update_rate)
        timeframe = 1
        drive = 0.0
        ddrive = (self.drive) / (self.update_rate * timeframe)
        while not rospy.is_shutdown():
            # Check for recent updates
            if self.corner_check():
                return "corner"
            elif self.doorway_check():
                return "doorway"

            rospy.loginfo(drive) 
            self.publish_cmd(drive, self.yaw_ce)
            self.ir_pid_pub.publish(self.ir_setpt_msg)
            rate.sleep()
            if drive < self.drive and self.revived:
                drive += ddrive
        return "completed"

    def yaw_control_effort_callback(self, msg):
        self.yaw_ce = - msg.data



            
