#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64

class Talker:
    def __init__(self):
        self.pub = rospy.Publisher('chatter', Float64, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        self.rate = rospy.Rate(1000) # 10hz
        self.talker()

    def talker(self):
        while not rospy.is_shutdown():
            hello_str = "hello world %s" % rospy.get_time()
            number = rospy.get_time()
            rospy.loginfo(number)
            self.pub.publish(number)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        start = Talker()
    except rospy.ROSInterruptException:
        pass