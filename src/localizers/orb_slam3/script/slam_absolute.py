#!/usr/bin/env python
import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import random
import numpy as np

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
lsy_old = None
height_old = None
yaw_offs_init = 0

# ransac variables
ransac_pairs = []
ransac_complete = False
ransac_err = 1
ransac_m_scale = 0


def perform_ransac(err, max_iter):
    global ransac_m_scale, ransac_err

    n_iter = 0
    while True:
        if n_iter > max_iter:
            break

        n_iter = n_iter + 1

        r_gt_y, r_sp_y = random.choice(ransac_pairs)
        if r_gt_y == 0 or r_sp_y == 0:
            continue

        new_ransac_m_scale = r_gt_y / r_sp_y

        err_curr = 0
        for gt_y, sp_y in ransac_pairs:
            if abs(gt_y - sp_y * new_ransac_m_scale) > err:
                err_curr = err_curr + 1

        err_curr /= float(len(ransac_pairs))

        if err_curr < ransac_err:
            ransac_m_scale = new_ransac_m_scale
            ransac_err = err_curr


# noinspection PyUnresolvedReferences,PyTypeChecker
def height_callback(height):
    global lsy, height_old, lsy_old, ransac_complete
    if ransac_complete is False and lsy is not None and last_yaw is not None:
        if height_old is None:
            height_old = height.data
            lsy_old = lsy
        else:
            ransac_pairs.append((height.data - height_old, lsy - lsy_old))

            if len(ransac_pairs) < 100:
                print("Collecting data for ransac. Frames: %d" % len(ransac_pairs))
            else:
                perform_ransac(0.08, 10000)
                print("SLAM RANSAC error: %f" % ransac_err)

            if ransac_err <= 0.2:
                global yaw_offs_init
                print("RANSAC scale: %f su/m" % ransac_m_scale)

                # determine yaw offset
                yaw_offs_init = last_yaw - lsyaw
                yaw_offs_init = np.arctan2(np.sin(yaw_offs_init), np.cos(yaw_offs_init))
                print("Yaw offset: %f deg" % ((yaw_offs_init * 180) / np.pi))

                ransac_complete = True

            lsy_old = lsy
            height_old = height.data
            lsy = None  # reset last slam y


def pose_callback(pose):
    if pose.pose.covariance[0] != 0:
        global lsy, lsyaw, lsy_old, height_old, yaw_offs_init, ransac_pairs, \
            ransac_complete, ransac_err, ransac_m_scale, lsx, lsz, lst
        # reset estimator
        lsy = None
        lsyaw = 0
        lsy_old = None
        height_old = None
        lsx = None
        lsz = None
        lst = None
        yaw_offs_init = 0
        ransac_pairs = []
        ransac_complete = False
        ransac_err = 1
        ransac_m_scale = 0
        print("Scale estimator reset")
        return

    if ransac_complete is True:
        global p_off_x, p_off_y

        lg_x = pose.pose.pose.position.x * ransac_m_scale
        lg_y = pose.pose.pose.position.z * ransac_m_scale

        if p_off_x is None or p_off_y is None:
            p_off_x = lg_x
            p_off_y = lg_y

        lg_x -= p_off_x
        lg_y -= p_off_y

        pose_absolute = PoseStamped()
        pose_absolute.header.stamp = rospy.get_rostime()
        pose_absolute.pose.position.y = - (np.cos(yaw_offs_init) * lg_x - np.sin(yaw_offs_init) * lg_y)
        pose_absolute.pose.position.x = + (np.sin(yaw_offs_init) * lg_x + np.cos(yaw_offs_init) * lg_y)
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
        global lsy, lsyaw
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
