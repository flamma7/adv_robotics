#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Empty
import smach
from magic_states.magic_state import Magic_State
import sys

class Straight(Magic_State):
    outcomes = ['completed', 'skid']
    is_skid = 0
    is_kill = 0
    def __init__(self, update_rate, drive):
        super(Straight, self).__init__(update_rate, self.outcomes)
        
        self.drive_setting = drive
        self.yaw = 0
        self.update = update_rate
        
        rospy.Subscriber('/is_kill', Empty, self.set_kill)
        rospy.Subscriber('/is_skid', Empty, self.set_skid)

    def execute(self):
        rate = rospy.Rate(self.update)
        while not rospy.is_shutdown():
            if self.skid_check():
                self.is_skid = 0
                return 'skid'
            elif self.kill_check():
                self.publish_cmd(0,0)
                return 'completed'
            self.publish_cmd(self.drive_setting, self.yaw)
            rate.sleep()

        return 'completed'

    def skid_check():
        return self.is_skid
    def kill_check():
        return self.is_kill

    def set_skid(self,msg):
        self.is_skid = 1
        return
    def set_kill(self,msg):
        self.is_kill = 1
        return

class Skid(smach.State):
    outcomes = ['completed', 'straight']
    is_straight = 0
    is_kill = 0

    def __init__(self, update_rate, drive):
        print "Skid outcomes: ", self.outcomes
        super(Skid, self).__init__(update_rate, self.outcomes)
        self.drive_setting = drive
        self.yaw = 100
        self.update = update_rate

        rospy.Subscriber('/is_kill', Empty, self.set_kill)
        rospy.Subscriber('/is_straight', Empty, self.set_straight)
        
    def execute(self):
        rate = rospy.Rate(update_rate)
        while not rospy.is_shutdown():
            if self.straight_check():
                self.is_straight = 0
                return 'straight'
            elif self.kill_check():
                self.publish_cmd(0,0)
                return 'completed'
            self.publish_cmd(self.drive, self.yaw)
            rate.sleep()
        self.publish_cmd(0,0)
        return 'completed'

    def set_kill(self,msg):
        self.is_kill = 1
        return
    def set_straight(self,msg):
        self.is_straight = 1
        return

    def straight_check(self):
        return self.is_straight
    def kill_check(self):
        return self.is_kill

rospy.init_node('state_machine')
sm_top = smach.StateMachine(['sm_completed'])

while not rospy.has_param('sm/update_hz') and not rospy.is_shutdown():
    pass
update_rate = rospy.get_param('sm/update_hz')
drive_set = rospy.get_param('straight/drive')

# load rosparams
with sm_top:
    
    smach.StateMachine.add('skid', Skid(update_rate,drive_set), transitions={'completed':'sm_completed', 'straight':'straight'})
    smach.StateMachine.add('straight', Straight(update_rate,drive_set), transitions={'completed':'sm_completed', 'skid' :'skid'})
    
try:
    outcome = sm_top.execute()    
    # TODO send stop to motors
except rospy.ROSInterruptException:
    # TODO send stop to motors
    sys.exit()
