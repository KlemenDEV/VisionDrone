#!/usr/bin/env python
import math

import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped as Twist

velocity_pub = None

SLAM = 3
OPTICAL_FLOW = 2
MOTION_SIMULATION = 1
DEAD_RECKONING = 0

current_mode = OPTICAL_FLOW


def publish_velocity(veltype, msg_orig):
    rospy.loginfo_throttle(1, veltype)

    pose = Twist()
    pose.header.stamp = rospy.get_rostime()
    pose.header.frame_id = veltype
    pose.twist.twist.linear.x = msg_orig.twist.twist.linear.x
    pose.twist.twist.linear.y = msg_orig.twist.twist.linear.y
    pose.twist.twist.linear.z = msg_orig.twist.twist.linear.z
    velocity_pub.publish(pose)


def slam_cb(msg):
    global current_mode

    if math.isnan(msg.twist.covariance[0]):
        current_mode = OPTICAL_FLOW
    else:
        current_mode = SLAM
        publish_velocity("slam", msg)


def optical_flow_cb(msg):
    global current_mode
    if current_mode <= OPTICAL_FLOW:
        if math.isnan(msg.twist.covariance[0]):
            current_mode = MOTION_SIMULATION
        else:
            current_mode = OPTICAL_FLOW
            publish_velocity("optical_flow", msg)


def motion_simulation_cb(msg):
    global current_mode
    if current_mode <= MOTION_SIMULATION:
        if math.isnan(msg.twist.covariance[0]):
            current_mode = DEAD_RECKONING
        else:
            current_mode = MOTION_SIMULATION
            publish_velocity("motion_simulation", msg)


def dead_reckoning_cb(msg):
    global current_mode
    if current_mode is DEAD_RECKONING:
        publish_velocity("dead_reckoning", msg)


if __name__ == '__main__':
    rospy.init_node('velocity_fusion')

    velocity_pub = rospy.Publisher('/estimate/velocity', Twist, queue_size=10)

    s1 = rospy.Subscriber("/orb_slam3/velocity_out", Twist, slam_cb, queue_size=10)
    s2 = rospy.Subscriber("/optical_flow/velocity_out", Twist, optical_flow_cb, queue_size=10)
    s3 = rospy.Subscriber("/motion_simulation/velocity_out", Twist, motion_simulation_cb, queue_size=10)
    s4 = rospy.Subscriber("/dead_reckoning/velocity_out", Twist, dead_reckoning_cb, queue_size=10)

    rospy.spin()
