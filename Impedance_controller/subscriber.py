#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

class RosPractice:

    def __init__(self):
        self.number = 1

        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("chatter", Float64, self.callback)

    def callback(self,data):
        self.number = data.data
        self.listener()        
 
    def listener(self):
        print self.number

if __name__ == '__main__':
    start = RosPractice()
    rospy.spin()
