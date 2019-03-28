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

# load rosparams
with sm_top:
    smach.StateMachine.add('straight', Straight(), transitions={'completed':'sm_completed', 'doorway' :'doorway', 'corner':'corner'})
    smach.StateMachine.add('doorway', Doorway(), transitions={'end_doorway':'straight'})
    smach.StateMachine.add('corner', Corner(), transitions={'end_corner':'straight'})
    
try:
    outcome = sm_top.execute()
    # TODO send stop to motors
except rospy.ROSInterruptException:
    # TODO send stop to motors
    sys.exit()
    
# open sm and add states & transitions
    
# execute

# Send stop to the motors

# class StateMachine:

#     def __init__(self):
#         rospy.loginfo("State Machine Initializing")
#         self.drivePub = rospy.Publisher("drive/control_effort", Float64, queue_size=10)
#         self.drive_effort = 8
#         self.timer = rospy.Timer(rospy.Duration(0.1), self.setpoint_callback)

#     def dist_callback(self, msg):
#         self.distance = msg.data

#     def setpoint_callback(self, msg):
#         new_msg = Float64()
#         new_msg.data = self.drive_effort
#         self.drivePub.publish(new_msg)

# def main():
#     rospy.init_node('state_machine')
#     sm = StateMachine()
#     rospy.spin()


# if __name__ == "__main__":
#     main()
