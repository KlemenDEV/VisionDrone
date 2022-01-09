#!/usr/bin/env python

import quadcopter
import rospy
from mavros_msgs.msg import RCOut
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
import math

quad = quadcopter.Quadcopter(
    {
        'q1': {
            'L': 0.25,
            'r': 0.09,
            'prop_size': [10, 5],
            'weight': 1.856
        }
    }
)

velocity_pub = None

height = 0

time_last = None
started = False


def height_cb(msg):
    global height
    height = msg.data


def orient_cb(msg):
    global time_last, velocity_pub, quad, started

    if height < 0.3 and not started:
        time_last = msg.header.stamp.secs
        started = True
        return

    dt = msg.header.stamp.secs - time_last

    quad.set_rotation(msg.orientation)
    quad.update(dt)

    linear_rate = quad.get_linear_rate('q1')

    pose = TwistWithCovarianceStamped()
    pose.header.stamp = rospy.get_rostime()
    pose.header.frame_id = "uav_velocity"
    pose.twist.twist.linear.x = linear_rate[0]
    pose.twist.twist.linear.y = linear_rate[1]
    pose.twist.twist.linear.z = 0

    rate_total = math.sqrt(pow(linear_rate[0], 2) + pow(linear_rate[1], 2))
    if rate_total > 60:
        pose.twist.covariance[0] = float("nan")

    velocity_pub.publish(pose)

    time_last = msg.header.stamp.secs


def rc_out_cb(msg):
    global quad

    quad.set_motor_speeds('q1', [
        msg.channels[0],
        msg.channels[1],
        msg.channels[2],
        msg.channels[3],
    ])


if __name__ == "__main__":
    rospy.init_node('simulator')

    velocity_pub = rospy.Publisher('/motion_simulation/velocity_out', TwistWithCovarianceStamped, queue_size=10)

    rc_out_sub = rospy.Subscriber('/mavros/rc/out', RCOut, rc_out_cb, queue_size=10)
    height_sub = rospy.Subscriber('/drone/height_estimate', Float64, height_cb, queue_size=10)
    orient_sub = rospy.Subscriber('/imu/9dof', Imu, orient_cb, queue_size=10)

    rospy.spin()
