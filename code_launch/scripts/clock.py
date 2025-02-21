#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String, Header, Time, Float64

def talker():
    rospy.init_node('clock', anonymous=True)
    pub = rospy.Publisher('clock', Float64, queue_size=100)
    rate = rospy.Rate(45) # 10hz
    while not rospy.is_shutdown():
        time = rospy.get_time() - 1704300000
        rounded = round(time, 6)
        rospy.loginfo(rounded)
        pub.publish(rounded)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass