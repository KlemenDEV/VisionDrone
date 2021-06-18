#!/usr/bin/env python

import rospy
from scipy import io
from geometry_msgs.msg import PoseStamped

import numpy as np

import os

data = np.zeros((0, 0))


def odom_cb(msg):
    global data
    data = np.append(data, np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]))


def shutdown():
    global data
    path = os.path.abspath(rospy.get_param("~outfile"))
    rospy.logwarn("Writing matlab file " + path + ", " + str(data.size) + " entries")
    io.savemat(path, mdict={rospy.get_param("~pose").replace("/", "_"): data})


if __name__ == '__main__':
    rospy.init_node('posetomatlab')

    odom_sub = rospy.Subscriber(rospy.get_param("~pose"), PoseStamped, odom_cb, queue_size=10)

    rospy.on_shutdown(shutdown)

    rospy.spin()
