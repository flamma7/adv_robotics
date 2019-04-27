#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import smach
import sys

from magic_states.straight import Straight
from magic_states.corner import Corner
from magic_states.doorway import Doorway

rospy.init_node('state_machine')
sm_top = smach.StateMachine(['sm_completed'])

while not rospy.has_param('sm/update_hz') and not rospy.is_shutdown():
    pass
update_rate = rospy.get_param('sm/update_hz')
hdrive = int(rospy.get_param('~hdrive'))
cdrive = int(rospy.get_param('~cdrive'))
wall_setpoint = float(rospy.get_param('wall_setpoint'))
# load rosparams
with sm_top:
    smach.StateMachine.add('straight', Straight(update_rate, hdrive, wall_setpoint), transitions={'completed':'sm_completed', 'doorway' :'doorway', 'corner':'corner'})
    smach.StateMachine.add('doorway', Doorway(update_rate, hdrive), transitions={'end_doorway':'straight'})
    smach.StateMachine.add('corner', Corner(update_rate, cdrive), transitions={'end_corner':'straight'})
    # smach.StateMachine.add('doorway', Doorway(update_rate, hdrive_set), transitions={'end_doorway':'sm_completed'})
try:
    outcome = sm_top.execute()    
    # TODO send stop to motors
except rospy.ROSInterruptException:
    # TODO send stop to motors
    sys.exit()
