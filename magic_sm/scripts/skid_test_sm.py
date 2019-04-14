#!/usr/bin/env python
from __future__ import division
import rospy
from std_msgs.msg import Float64, Empty
import smach
from magic_states.magic_state import Magic_State
import sys

class Straight(Magic_State):
    outcomes = ['completed', 'skid']
    is_skid = 0
    is_kill = 0
    def __init__(self, update_rate, drive, wall_setpoint):
        super(Straight, self).__init__(update_rate, self.outcomes)
        self.drive_setting = drive
        self.yaw = 0
        self.wall_msg = Float64()
        self.wall_msg.data = wall_setpoint
        self.wall_pub = rospy.Publisher('yaw/setpoint', Float64, queue_size=10)
        rospy.Subscriber('yaw/control_effort', Float64, self.yaw_ce_callback)
        rospy.Subscriber('/is_kill', Empty, self.set_kill)
        rospy.Subscriber('/is_skid', Empty, self.set_skid)

    def execute(self, userdata):
        self.is_skid = False
        rospy.logwarn("Executing Straight")
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            if self.skid_check():
                self.is_skid = 0
                return 'skid'
            elif self.kill_check():
                self.publish_cmd(0,0)
                return 'completed'
            self.wall_pub.publish(self.wall_msg)
#            rospy.loginfo("Sending command:"+ str(self.drive_setting)+"+"+str(self.yaw))
            self.publish_cmd(self.drive_setting, self.yaw)
            rate.sleep()
        return 'completed'

    def skid_check(self):
        return self.is_skid
    def kill_check(self):
        return self.is_kill

    def set_skid(self,msg):
        self.is_skid = 1
    def set_kill(self,msg):
        self.is_kill = 1

    def yaw_ce_callback(self, msg):
        self.yaw = -msg.data

class Skid(Magic_State):
    outcomes = ['completed', 'straight']
    is_straight = 0
    is_kill = 0

    def __init__(self, update_rate, drive):
        print "Skid outcomes: ", self.outcomes
        super(Skid, self).__init__(update_rate, self.outcomes)
        self.drive_setting = drive
        self.yaw = 100 # -100? 

        rospy.Subscriber('/is_kill', Empty, self.set_kill)
        rospy.Subscriber('/is_straight', Empty, self.set_straight)
        
    def execute(self, userdata):
        self.is_straight = False
        rospy.logwarn("Executing Skid")
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            if self.straight_check():
                self.is_straight = 0
                return 'straight'
            elif self.kill_check():
                self.publish_cmd(0,0)
                return 'completed'
            self.publish_cmd(self.drive_setting, self.yaw)
            rate.sleep()
        self.publish_cmd(0,0)
        return 'completed'

    def set_kill(self,msg):
        self.is_kill = 1

    def set_straight(self,msg):
        self.is_straight = 1

    def straight_check(self):
        return self.is_straight
    def kill_check(self):
        return self.is_kill

rospy.init_node('state_machine')
sm_top = smach.StateMachine(['sm_completed'])

while not rospy.has_param('sm/update_hz') and not rospy.is_shutdown():
    pass

update_rate = float(rospy.get_param('sm/update_hz'))
drive_set = int(rospy.get_param('~straight/drive'))
wall_setpoint = float(rospy.get_param('wall_setpoint'))

# load rosparams
with sm_top:
    
    smach.StateMachine.add('Straight', Straight(update_rate,drive_set, wall_setpoint), transitions={'completed':'sm_completed', 'skid' :'Skid'})
    smach.StateMachine.add('Skid', Skid(update_rate,drive_set), transitions={'completed':'sm_completed', 'straight':'Straight'})

    
try:
    outcome = sm_top.execute()    
    # TODO send stop to motors
except rospy.ROSInterruptException:
    # TODO send stop to motors
    sys.exit()
