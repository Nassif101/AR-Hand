#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Quaternion
import tf
import math

def callback(msg):
    global calibrated
    global quatref
    if calibrated is True:
        quatref = [msg.x, msg.y, msg.z, msg.w]
        calibrated = False
    quatdiff = tf.transformations.quaternion_multiply(tf.transformations.quaternion_inverse(quatref), [msg.x, msg.y, msg.z, msg.w])    
    pub.publish(Quaternion(quatdiff[0], quatdiff[1], quatdiff[2], quatdiff[3]))
       

def main():
    rospy.init_node("quatref2", anonymous=True)
    global calibrated
    calibrated = True
    rospy.Subscriber("/ringPIP", Quaternion, callback)
    global pub
    pub = rospy.Publisher("/quatref2", Quaternion, queue_size=100)
    rospy.spin()

if __name__ == "__main__":
    main()