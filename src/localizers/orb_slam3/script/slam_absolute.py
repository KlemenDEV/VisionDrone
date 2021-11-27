#!/usr/bin/env python
import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import numpy as np
import random

pose_pub = None
vel_pub = None
vel_enu_pub = None
p_off_x = None
p_off_y = None

# slam last values
lsy = None
lsyaw = 0

# slam old values
lsx = None
lsz = None
lst = None

# imu last yaw
last_yaw = None

# init data
yaw_offs_init = 0

# ransac variables
ransac_pairs = []
ransac_complete = False
ransac_err = 1
ransac_m_k = 0
ransac_m_n = 0


def perform_ransac(err, iter_count):
    global ransac_m_k, ransac_m_n, ransac_err

    for _ in range(iter_count):
        pts = random.sample(ransac_pairs, 2)

        y1, x1 = pts[0]
        y2, x2 = pts[1]

        if x2 - x1 == 0:  # division by 0 protection
            continue

        new_ransac_m_k = (y2 - y1) / (x2 - x1)
        new_ransac_m_n = y1 - new_ransac_m_k * x1

        err_curr = 0
        for y, x in ransac_pairs:
            if abs(new_ransac_m_k * x + new_ransac_m_n - y) > err:
                err_curr = err_curr + 1
        err_curr /= float(len(ransac_pairs))

        if err_curr < ransac_err:
            ransac_m_k = new_ransac_m_k
            ransac_m_n = new_ransac_m_n
            ransac_err = err_curr


# noinspection PyUnresolvedReferences,PyTypeChecker
def height_callback(height):
    global lsy, ransac_complete
    if ransac_complete is False and lsy is not None and last_yaw is not None:
        ransac_pairs.append((height.data, lsy))

        if len(ransac_pairs) < 60:
            print("Collecting data for ransac. Frames: %d" % len(ransac_pairs))
        else:
            perform_ransac(0.18, 5000)
            print("SLAM RANSAC error: %f" % ransac_err)

        if ransac_err <= 0.25:
            global yaw_offs_init
            print("RANSAC scale: %f su/m" % ransac_m_k)

            # determine yaw offset
            yaw_offs_init = lsyaw
            print("Yaw offset: %f deg" % ((yaw_offs_init * 180) / np.pi))

            ransac_complete = True

        lsy = None  # reset last slam y


def pose_callback(pose):
    global ransac_pairs, lsyaw, yaw_offs_init, p_off_x, p_off_y, \
        ransac_complete, ransac_err, ransac_m_k, lsx, lsz, lst, lsy

    # reset estimator if invalid data
    if pose.pose.covariance[0] != 0:
        lsy = None
        lsyaw = 0
        lsx = None
        lsz = None
        lst = None
        p_off_x = None
        p_off_y = None
        yaw_offs_init = 0
        ransac_pairs = []
        ransac_complete = False
        ransac_err = 1
        ransac_m_k = 0
        return

    if ransac_complete is True:
        lg_x = pose.pose.pose.position.x * ransac_m_k
        lg_y = pose.pose.pose.position.z * ransac_m_k

        lg_x_rotated = -(np.cos(-yaw_offs_init) * lg_x - np.sin(-yaw_offs_init) * lg_y)
        lg_y_rotated = +(np.sin(-yaw_offs_init) * lg_x + np.cos(-yaw_offs_init) * lg_y)

        if p_off_x is None or p_off_y is None:
            p_off_x = lg_x_rotated
            p_off_y = lg_y_rotated

        lg_x = lg_x_rotated - p_off_x
        lg_y = lg_y_rotated - p_off_y

        pose_absolute = PoseStamped()
        pose_absolute.header.stamp = rospy.get_rostime()
        pose_absolute.pose.position.x = lg_y
        pose_absolute.pose.position.y = lg_x
        pose_pub.publish(pose_absolute)

        if lsz is None:
            lsx = pose_absolute.pose.position.x
            lsz = pose_absolute.pose.position.y
            lst = pose.header.stamp.to_sec()
        else:
            dx = pose_absolute.pose.position.x - lsx
            dy = pose_absolute.pose.position.y - lsz
            dt = pose.header.stamp.to_sec() - lst

            if dt > 0:
                vx_enu = dx / dt
                vy_enu = dy / dt
                vel_enu = TwistWithCovarianceStamped()
                vel_enu.header.stamp = rospy.get_rostime()
                vel_enu.header.frame_id = "uav_velocity_enu"
                vel_enu.twist.twist.linear.x = vx_enu
                vel_enu.twist.twist.linear.y = vy_enu
                vel_enu.twist.twist.linear.z = 0
                vel_enu_pub.publish(vel_enu)

                vx = np.cos(-last_yaw) * vx_enu - np.sin(-last_yaw) * vy_enu
                vy = np.sin(-last_yaw) * vx_enu + np.cos(-last_yaw) * vy_enu
                vel = TwistWithCovarianceStamped()
                vel.header.stamp = rospy.get_rostime()
                vel.header.frame_id = "uav_velocity"
                vel.twist.twist.linear.x = vx
                vel.twist.twist.linear.y = vy
                vel.twist.twist.linear.z = 0
                vel_pub.publish(vel)

            lsx = pose_absolute.pose.position.x
            lsz = pose_absolute.pose.position.y
            lst = pose.header.stamp.to_sec()
    else:
        lsy = -pose.pose.pose.position.y
        (_, _, lsyaw) = tf.transformations.euler_from_quaternion([
            pose.pose.pose.orientation.x,
            pose.pose.pose.orientation.y,
            pose.pose.pose.orientation.z,
            pose.pose.pose.orientation.w,
        ])


def orient_cb(msg):
    global last_yaw
    (_, _, last_yaw) = tf.transformations.euler_from_quaternion([
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w,
    ])


if __name__ == "__main__":
    rospy.init_node('simulator')

    pose_pub = rospy.Publisher('/estimate/pose_raw', PoseStamped, queue_size=5)
    vel_pub = rospy.Publisher('/estimate/velocity', TwistWithCovarianceStamped, queue_size=5)
    vel_enu_pub = rospy.Publisher('/estimate/velocity_enu', TwistWithCovarianceStamped, queue_size=5)

    pose_sub = rospy.Subscriber('/orb_slam3/pose_out', PoseWithCovarianceStamped, pose_callback, queue_size=5)
    height_sub = rospy.Subscriber('/drone/height_estimate', Float64, height_callback, queue_size=5)
    orient_sub = rospy.Subscriber('/imu/9dof', Imu, orient_cb, queue_size=15)

    rospy.spin()
