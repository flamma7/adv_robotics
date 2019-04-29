#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, Bool

class Detector:
    def __init__(self):

        self.side_thresh = float(rospy.get_param('ir/side_thresh'))
        self.front_thresh = float(rospy.get_param('ir/front_thresh'))

        self.door_pub = rospy.Publisher('doorway', Bool, queue_size=1)
        self.corner_pub = rospy.Publisher('corner', Bool, queue_size=1)

        rospy.Subscriber('/front_distance', Float64, self.front_callback)
        rospy.Subscriber('/distance', Float64, self.side_callback)
        self.update_rate = int(rospy.get_param('sm/update_hz', 50))

        rospy.wait_for_message("/front_distance", Float64)
        rospy.wait_for_message("/distance", Float64)

    def execute(self):
        doorway_start = False
        corner_start = False
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            # Check corner
            msg = Bool()
            if self.front_ir < self.front_thresh:
                msg.data = True
                if not corner_start:
                    rospy.loginfo("Corner detected")                    
                corner_start = True
            else:
                msg.data = False
                if corner_start == True:
                    rospy.logwarn("Corner ended")
                    corner_start = False
                    rospy.loginfo(self.front_ir)
                    rospy.loginfo(self.side_ir)
            self.corner_pub.publish(msg)

            # Check doorway
            msg = Bool()
            if self.front_ir > self.front_thresh and self.side_ir > self.side_thresh:
                msg.data = True
                if not doorway_start:
                    rospy.logerr("Doorway detected")                    
                doorway_start = True
            else:
                msg.data = False
                if doorway_start == True:
                    rospy.logerr("Doorway ended")
                    doorway_start = False
                    rospy.loginfo(self.front_ir)
                    rospy.loginfo(self.side_ir)
            self.door_pub.publish(msg)
            rate.sleep()

    def front_callback(self, msg):
        self.front_ir = msg.data

    def side_callback(self, msg):
        self.side_ir = msg.data
        
if __name__ == "__main__":
    rospy.init_node('detector')
    d = Detector()
    d.execute()
