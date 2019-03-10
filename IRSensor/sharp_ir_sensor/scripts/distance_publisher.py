#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sharp_ir_sensor import PololuController

def sharp_ir_sensor():
    pub = rospy.Publisher('distance', Float64, queue_size=10)
    rospy.init_node('sharp_ir_sensor', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    maestro = PololuController()
    while not rospy.is_shutdown():
        V = maestro.getPosition(channel=args.channel)
        m = 0.0008073863884277423
        b = -0.20900036625160207
        k = 0.43
        distance = (m/(V + b)) - k
        rospy.loginfo(distance)
        pub.publish(distance)
        rate.sleep()

if __name__ == '__main__':
    try:
        sharp_ir_sensor()
    except rospy.ROSInterruptException:
        pass