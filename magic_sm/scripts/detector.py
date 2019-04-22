#!/usr/bin/env python

class Detector:
    def __init__(self):

        self.side_thresh = float(rospy.get_param('ir/side_thresh'))
        self.front_thresh = float(rospy.get_param('ir/front_thresh'))

        self.door_pub = ropspy.Publisher('doorway', Bool, queue_size=10)
        self.corner_pub = ropspy.Publisher('corner', Bool, queue_size=10)

        rospy.Subscriber('/')
