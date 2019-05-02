#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import smach
import sys

from magic_states.runway2 import Runway
from magic_states.airborne import Airborne
from magic_states.landing import Landing

rospy.init_node('state_machine')
sm_top = smach.StateMachine(['sm_completed'])

while not rospy.has_param('sm/update_hz') and not rospy.is_shutdown():
    pass

dist = float(rospy.get_param('dist'))
update_rate = int(rospy.get_param('sm/update_hz'))
jump_angle = float(rospy.get_param('jump_angle'))
landing_time = float(rospy.get_param('landing_time'))
landing_speed = int(rospy.get_param('landing_speed'))

# load rosparams
with sm_top:
    smach.StateMachine.add('runway', Runway(update_rate, dist, jump_angle), transitions={'airborne':'airborne'})
    smach.StateMachine.add('airborne', Airborne(update_rate, jump_angle), transitions={'landing':'landing'})
    smach.StateMachine.add('landing', Landing(update_rate, landing_speed, landing_time), transitions={'completed':'sm_completed'})
try:
    outcome = sm_top.execute()    
    # TODO send stop to motors
except rospy.ROSInterruptException:
    # TODO send stop to motors
    sys.exit()
