#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu

ax = 0
ay = 0
az = 0
gx = 0
gy = 0
gz = 0
n = 0


def orient_cb(msg):
    global n, ax, ay, az, gx, gy, gz

    ax = ax + msg.linear_acceleration.x
    ay = ay + msg.linear_acceleration.y
    az = az + msg.linear_acceleration.z

    gx = gx + msg.angular_velocity.x
    gy = gy + msg.angular_velocity.y
    gz = gz + msg.angular_velocity.z

    n = n + 1

    pass


if __name__ == "__main__":
    rospy.init_node('tmp')

    orient_sub = rospy.Subscriber('/imu/9dof', Imu, orient_cb, queue_size=15)

    rospy.spin()

    print("Acc bias x: %f y: %f z: %f" % (ax / n, ay / n, az / n))
    print("Gyro bias x: %f y: %f z: %f" % (gx / n, gy / n, gz / n))
