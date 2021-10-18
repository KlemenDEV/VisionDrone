#!/usr/bin/env python

import rospy
import tf.transformations
from scipy import io
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped
from sensor_msgs.msg import Imu

from velocity_integrator.srv import SetDatum

import numpy as np

import math

import os
import time

data = np.empty((0, 4), float)
velocitydata = np.empty((0, 4), float)
velocitysource = np.empty((0, 1), dtype='object')

gtdata = np.empty((0, 4), float)
gtvelocitydata = np.empty((0, 4), float)
gtvelocitydatalocal = np.empty((0, 4), float)

start_time = None
datum = None

gtox = 0
gtoy = 0
gtoz = 0
gttime = None

yaw = 0


def odom_cb(msg):
    global data, start_time
    if start_time is None:
        start_time = time.time()

    data = np.vstack(
        (data, np.array([time.time() - start_time, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])))


def gt_cb(msg):
    global gtdata, start_time, gtox, gtoy, gtoz, gttime, gtvelocitydata, gtvelocitydatalocal

    if start_time is not None:
        gtdata = np.vstack((gtdata, np.array(
            [time.time() - start_time, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])))

        if gttime is not None:
            dt = time.time() - gttime
            vx = (msg.pose.position.x - gtox) / dt
            vy = (msg.pose.position.y - gtoy) / dt
            vz = (msg.pose.position.z - gtoz) / dt

            gtvelocitydata = np.vstack((gtvelocitydata, np.array([time.time() - start_time, vx, vy, vz])))

            vxl = math.cos(yaw) * vy - math.sin(yaw) * vx
            vyl = math.sin(yaw) * vy + math.cos(yaw) * vx

            gtvelocitydatalocal = np.vstack((gtvelocitydatalocal, np.array([time.time() - start_time, vxl, vyl, vz])))

        gtox = msg.pose.position.x
        gtoy = msg.pose.position.y
        gtoz = msg.pose.position.z
        gttime = time.time()


def imu_cb(msg):
    global yaw

    quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]


def velocity_cb(msg):
    global velocitydata, start_time, velocitysource
    if start_time is not None:
        velocitysource = np.vstack((velocitysource, np.array(msg.header.frame_id, dtype='object')))
        velocitydata = np.vstack((velocitydata, np.array(
            [time.time() - start_time, msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])))


def shutdown():
    global data, gtdata, gtvelocitydata, gtvelocitydatalocal, datum, velocitydata, velocitysource
    path = os.path.abspath(rospy.get_param("~outfile"))
    rospy.logwarn("Writing matlab file " + path + ", " + str(data.size) + " data entries")
    io.savemat(path, mdict={
        'data': data,
        'gt': gtdata,
        'gtvelocity': gtvelocitydata,
        'gtvelocitylocal': gtvelocitydatalocal,
        'datum': datum,
        'velocity': velocitydata,
        'velocitysource': velocitysource,
    })


def handle_datum(req):
    global datum
    datum = np.array([req.geo_pose.position.latitude, req.geo_pose.position.longitude, req.geo_pose.position.altitude])
    return []


if __name__ == '__main__':
    rospy.init_node('posetomatlab')

    odom_sub = rospy.Subscriber(rospy.get_param("~pose"), PoseStamped, odom_cb, queue_size=10)
    velocity_sub = rospy.Subscriber(rospy.get_param("~velocity"), TwistWithCovarianceStamped, velocity_cb,
                                    queue_size=10)

    odom_sub2 = rospy.Subscriber(rospy.get_param("~gt"), PoseStamped, gt_cb, queue_size=10)
    imu_sub = rospy.Subscriber(rospy.get_param("~imu_data"), Imu, imu_cb, queue_size=10)

    rospy.Service('datum_ref', SetDatum, handle_datum)

    rospy.on_shutdown(shutdown)

    rospy.spin()
