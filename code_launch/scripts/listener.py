#!/usr/bin/env python3
import rospy
from quatid_msg.msg import QuatID
from std_msgs.msg import Header, Time, Float64
  
time = None
latency_sum = 0
sample_count = 0
def _callback(msg):
    global time
    time = msg.data
    # rospy.loginfo("clock : %s", time)

def callback(msg):
    global time
    global latency_sum
    global sample_count
    if time is not None:
        stamp = msg.stamp
        latency = time - stamp.data
        latency_sum += latency
        sample_count += 1
        # if sample_count == 10:
        #     rospy.loginfo("latency average: %s", latency_sum*1000/sample_count)
        #     latency_sum = 0
        #     sample_count = 0
        # rospy.loginfo("clock: %s, stamp: %s, latency: %s", time, stamp.data, latency)
        rospy.loginfo("time: %s", time)

# def callback_timestamp(msg):
#     global time
#     # _time = rospy.get_time() - 1701700000
#     global latency_sum
#     global sample_count
#     if time is not None:
#         stamp = msg.data
#         latency = time - stamp
#         latency_sum += latency
#         sample_count += 1
#         if sample_count == 10:
#             rospy.loginfo("latency: %s", latency_sum*1000/sample_count)
#             latency_sum = 0
#             sample_count = 0
        
        # rospy.loginfo("clock: %s, stamp: %s, latency: %s", time, stamp.data, latency)
        # rospy.loginfo("latency: %s", latency*1000)
    # stamp = msg.data
    # latency = _time - stamp
    # rospy.loginfo("latency: %s", latency*1000)
def main():
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber("/clock", Float64, _callback)
    rospy.Subscriber("/quatid_msg", QuatID, callback)
    # rospy.Subscriber("/timestamp", Float64, callback_timestamp)
    rospy.spin()

if __name__ == "__main__":
    main()
