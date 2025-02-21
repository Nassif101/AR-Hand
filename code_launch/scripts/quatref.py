#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Quaternion
import tf
import math

ring1_pub = rospy.Publisher("/ring1", Quaternion, queue_size=100) 
ring2_pub = rospy.Publisher("/ring2", Quaternion, queue_size=100)
palm_pub = rospy.Publisher("/palm", Quaternion, queue_size=100)


def ring1_callback(msg):
    global calibrated
    global quatref
    if calibrated is True:
        quatref = [msg.x, msg.y, msg.z, msg.w]
        calibrated = False
    quatdiff = tf.transformations.quaternion_multiply(tf.transformations.quaternion_inverse(quatref), [msg.x, msg.y, msg.z, msg.w])  
    ring1_pub.publish(Quaternion(quatdiff[0], quatdiff[1], quatdiff[2], quatdiff[3]))
       
def ring2_callback(msg):
    global calibrated2
    global quatref2
    if calibrated2 is True:
        quatref2 = [msg.x, msg.y, msg.z, msg.w]
        calibrated2 = False
    quatdiff2 = tf.transformations.quaternion_multiply(tf.transformations.quaternion_inverse(quatref2), [msg.x, msg.y, msg.z, msg.w])  
    ring2_pub.publish(Quaternion(quatdiff2[0], quatdiff2[1], quatdiff2[2], quatdiff2[3]))

def palm_callback(msg):
    global calibrated3
    global quatref3
    if calibrated3 is True:
        quatref3 = [msg.x, msg.y, msg.z, msg.w]
        calibrated3 = False
    quatdiff3 = tf.transformations.quaternion_multiply(tf.transformations.quaternion_inverse(quatref3), [msg.x, msg.y, msg.z, msg.w])  
    palm_pub.publish(Quaternion(quatdiff3[0], quatdiff3[1], quatdiff3[2], quatdiff3[3]))

def main():
    rospy.init_node("quatref", anonymous=True)
    global calibrated
    calibrated = True
    global calibrated2
    calibrated2 = True
    global calibrated3
    calibrated3 = True
    rospy.Subscriber("/ringMCP", Quaternion, ring1_callback)
    rospy.Subscriber("/ringPIP", Quaternion, ring2_callback)
    rospy.Subscriber("/palm_angle", Quaternion, palm_callback)
    rospy.spin()

if __name__ == "__main__":
    main()