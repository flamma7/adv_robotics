import rospy
from std_msgs.msg import Float64

class Bangbang():

    def __init__(self):
        self.control_constant = 40
        self.thresh1 = rospy.get_param('thresh1', 5)
        self.thresh2 = rospy.get_param('thresh2', 8)

        rospy.Subscriber('/distance', Float64, self.distance_callback)

        self.control_pub = rospy.Publisher('hall/control_effort', Float64, queue_size=1)


    def distance_callback(self,msg):
        dist = msg.data
        ce = Float64()
        if dist < self.thresh1:
            ce.data = self.control_constant
        elif dist > self.thresh2:
            ce.data = -1 * self.control_constant
        else:
            ce.data = 0
        self.control_pub.publish(ce)

def main():
    rospy.init_node("bangbang")
    bang = Bangbang()
    rospy.spin()
        

if __name__ == '__main__':
    main()

    
