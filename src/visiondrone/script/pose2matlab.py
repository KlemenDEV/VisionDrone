#!/usr/bin/env python

import rospy
from scipy import io
from geometry_msgs.msg import PoseStamped

import numpy as np

import os

data = np.zeros((0, 0))
gtdata = np.zeros((0, 0))


def odom_cb(msg):
    global data
    data = np.append(data, np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]))


def gt_cb(msg):
    global gtdata
    gtdata = np.append(gtdata, np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]))


def shutdown():
    global data
    path = os.path.abspath(rospy.get_param("~outfile"))
    rospy.logwarn("Writing matlab file " + path + ", " + str(data.size) + " data entries")
    io.savemat(path, mdict={
        'data': data,
        'gt': gtdata
    })


if __name__ == '__main__':
    rospy.init_node('posetomatlab')

    odom_sub = rospy.Subscriber(rospy.get_param("~pose"), PoseStamped, odom_cb, queue_size=10)
    odom_sub2 = rospy.Subscriber(rospy.get_param("~gt"), PoseStamped, gt_cb, queue_size=10)

    rospy.on_shutdown(shutdown)

    rospy.spin()
