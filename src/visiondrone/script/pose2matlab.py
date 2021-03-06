#!/usr/bin/env python
import rospy
import tf.transformations
from scipy import io
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from ublox_msgs.msg import NavPVT

from velocity_integrator.srv import SetDatum

import numpy as np

import math

import os

estimate_pose = np.empty((0, 4), float)
estimate_vel = np.empty((0, 4), float)
estimate_vel_enu = np.empty((0, 4), float)

gt_pose = np.empty((0, 4), float)
gt_vel = np.empty((0, 4), float)
gt_vel_enu = np.empty((0, 4), float)

gt_hacc = np.empty((0, 2), float)
gt_vacc = np.empty((0, 2), float)
gt_sacc = np.empty((0, 2), float)

h_rel = np.empty((0, 2), float)
h_gnd = np.empty((0, 2), float)

current_time = None
start_time = None
datum = None
yaw = 0


def odom_cb(msg):
    if current_time is None:
        return

    global estimate_pose
    estimate_pose = np.vstack(
        (estimate_pose,
         np.array([current_time, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])))


def gt_navpvt_cb(msg):
    if current_time is None:
        return

    global gt_hacc, gt_vacc, gt_sacc
    gt_hacc = np.vstack((gt_hacc, np.array([current_time, msg.hAcc])))
    gt_vacc = np.vstack((gt_vacc, np.array([current_time, msg.vAcc])))
    gt_sacc = np.vstack((gt_sacc, np.array([current_time, msg.sAcc])))


def height_rel(msg):
    if current_time is None:
        return

    global h_rel
    h_rel = np.vstack((h_rel, np.array([current_time, msg.data])))


def height_gnd(msg):
    if current_time is None:
        return

    global h_gnd
    h_gnd = np.vstack((h_gnd, np.array([current_time, msg.data])))


def velocity_cb(msg):
    if current_time is None:
        return

    global estimate_vel
    estimate_vel = np.vstack((estimate_vel, np.array(
        [current_time, msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])))


def velocity_enu_cb(msg):
    if current_time is None:
        return

    global estimate_vel_enu
    estimate_vel_enu = np.vstack((estimate_vel_enu, np.array(
        [current_time, msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])))


def gt_cb(msg):
    if current_time is None:
        return

    global gt_pose
    gt_pose = np.vstack((gt_pose, np.array(
        [current_time, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])))


def gt_velocity_cb(msg):
    if current_time is None:
        return

    global gt_vel_enu, gt_vel
    gt_vel_enu = np.vstack((gt_vel_enu, np.array(
        [current_time, msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])))

    vxl = math.cos(-yaw) * msg.twist.twist.linear.x - math.sin(-yaw) * msg.twist.twist.linear.y
    vyl = math.sin(-yaw) * msg.twist.twist.linear.x + math.cos(-yaw) * msg.twist.twist.linear.y
    gt_vel = np.vstack(
        (gt_vel, np.array([current_time, vxl, vyl, msg.twist.twist.linear.z])))


def imu_cb(msg):
    global yaw, current_time, start_time

    if start_time is None:
        start_time = msg.header.stamp.to_sec()
        current_time = 0
    else:
        current_time = msg.header.stamp.to_sec() - start_time

    quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]


def handle_datum(req):
    global datum
    datum = np.array([req.geo_pose.position.latitude, req.geo_pose.position.longitude, req.geo_pose.position.altitude])
    return []


def shutdown():
    path = os.path.abspath(rospy.get_param("~outfile"))
    io.savemat(path, mdict={
        'datum': datum,

        'gt_pose': gt_pose,
        'gt_vel': gt_vel,
        'gt_vel_enu': gt_vel_enu,
        'gt_hacc': gt_hacc,
        'gt_vacc': gt_vacc,
        'gt_sacc': gt_sacc,

        'estimate_pose': estimate_pose,
        'estimate_vel': estimate_vel,
        'estimate_vel_enu': estimate_vel_enu,

        'h_rel': h_rel,
        'h_gnd': h_gnd
    })
    rospy.logwarn("Writing matlab file " + path + ", " + str(estimate_pose.size) + " data entries")


if __name__ == '__main__':
    rospy.init_node('posetomatlab')

    # estimate subscribers
    estimate_pose_sub = rospy.Subscriber('/estimate/pose', PoseStamped, odom_cb, queue_size=10)
    estimate_vel_sub = rospy.Subscriber('/estimate/velocity', TwistWithCovarianceStamped, velocity_cb, queue_size=10)
    estimate_vel_enu_sub = rospy.Subscriber('/estimate/velocity_enu', TwistWithCovarianceStamped, velocity_enu_cb,
                                            queue_size=10)

    # height data subscribers
    height_rel_sub = rospy.Subscriber('/drone/height_estimate', Float64, height_rel, queue_size=10)
    height_gnd_sub = rospy.Subscriber('/drone/height_ground', Float64, height_gnd, queue_size=10)

    # gt subscribers
    gt_pose_sub = rospy.Subscriber('/ublox/fix/local', PoseStamped, gt_cb, queue_size=10)
    gt_vel_enu_sub = rospy.Subscriber('/ublox/fix_velocity', TwistWithCovarianceStamped, gt_velocity_cb, queue_size=10)
    gt_navpvt_sub = rospy.Subscriber('/ublox/navpvt', NavPVT, gt_navpvt_cb, queue_size=10)

    # misc data subscribers and services
    imu_sub = rospy.Subscriber('/imu/9dof', Imu, imu_cb, queue_size=10)
    rospy.Service('datum_ref', SetDatum, handle_datum)

    rospy.on_shutdown(shutdown)

    rospy.spin()
