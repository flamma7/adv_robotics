#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import smach
import sys

from magic_states.straight import Straight
from magic_states.skid import Skid
from magic_states.doorway import Doorway

class Straight(smach.State):
    outcomes = ['completed', 'skid']
    def __init__(self, drive):
        self.drive_setting = drive
        self.yaw = 0

    def execute(self):
        while not rospy.is_shutdown():
            if self.skid_check():
                return 'skid'
            if self.kill_check():
                return 'completed'
            self.publish_cmd(self.drive_setting, self.yaw)

        return 'completed'

rospy.init_node('state_machine')
sm_top = smach.StateMachine(['sm_completed'])

while not rospy.has_param('sm/update_hz') and not rospy.is_shutdown():
    pass
update_rate = rospy.get_param('sm/update_hz')

# load rosparams
with sm_top:
    smach.StateMachine.add('straight', Straight(update_rate), transitions={'completed':'sm_completed', 'doorway' :'doorway', 'corner':'corner'})
    smach.StateMachine.add('doorway', Doorway(update_rate), transitions={'end_doorway':'straight'})
    smach.StateMachine.add('corner', Corner(update_rate), transitions={'end_corner':'straight'})
    
try:
    outcome = sm_top.execute()    
    # TODO send stop to motors
except rospy.ROSInterruptException:
    # TODO send stop to motors
    sys.exit()
