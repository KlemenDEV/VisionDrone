#!/usr/bin/env python

import rospy
from scipy import io
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped

from velocity_integrator.srv import SetDatum

import numpy as np

import os
import time

data = np.empty((0, 4), float)
velocitydata = np.empty((0, 4), float)
gtdata = np.empty((0, 4), float)

start_time = None
datum = None


def odom_cb(msg):
    global data, start_time
    if start_time is None:
        start_time = time.time()

    data = np.vstack(
        (data, np.array([time.time() - start_time, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])))


def gt_cb(msg):
    global gtdata, start_time
    if start_time is not None:
        gtdata = np.vstack((gtdata, np.array(
            [time.time() - start_time, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])))


def velocity_cb(msg):
    global velocitydata, start_time
    if start_time is not None:
        velocitydata = np.vstack((velocitydata, np.array(
            [time.time() - start_time, msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])))


def shutdown():
    global data
    path = os.path.abspath(rospy.get_param("~outfile"))
    rospy.logwarn("Writing matlab file " + path + ", " + str(data.size) + " data entries")
    io.savemat(path, mdict={
        'data': data,
        'gt': gtdata,
        'datum': datum,
        'velocity': velocitydata
    })


def handle_datum(req):
    global datum
    datum = np.array([req.geo_pose.position.latitude, req.geo_pose.position.longitude, req.geo_pose.position.altitude])
    print(req)
    return []


if __name__ == '__main__':
    rospy.init_node('posetomatlab')

    odom_sub = rospy.Subscriber(rospy.get_param("~pose"), PoseStamped, odom_cb, queue_size=10)
    velocity_sub = rospy.Subscriber(rospy.get_param("~velocity"), TwistWithCovarianceStamped, velocity_cb,
                                    queue_size=10)
    odom_sub2 = rospy.Subscriber(rospy.get_param("~gt"), PoseStamped, gt_cb, queue_size=10)

    rospy.Service('datum_ref', SetDatum, handle_datum)

    rospy.on_shutdown(shutdown)

    rospy.spin()
