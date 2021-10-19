#!/usr/bin/env python
import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import random
import numpy as np

pose_pub = None
p_off_x = None
p_off_y = None

# slam last values
lsy = None
lsyaw = 0

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


def perform_ransac():
    global ransac_m_scale, ransac_err

    if len(ransac_pairs) < 100:
        return

    n_iter = 0
    while True:
        if n_iter > 20000:
            break

        n_iter = n_iter + 1

        r_gt_y, r_sp_y = random.choice(ransac_pairs)
        if r_gt_y == 0 or r_sp_y == 0:
            continue

        new_ransac_m_scale = r_gt_y / r_sp_y

        err_curr = 0
        for gt_y, sp_y in ransac_pairs:
            if abs(gt_y - sp_y * new_ransac_m_scale) > 2:
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
            perform_ransac()

            if ransac_err < 1:
                print("SLAM RANSAC error: %f" % ransac_err)
            else:
                print("Ransac in progress. Frames: %d" % len(ransac_pairs))

            if ransac_err <= 0.3:
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
            ransac_complete, ransac_err, ransac_m_scale
        # reset estimator
        lsy = None
        lsyaw = 0
        lsy_old = None
        height_old = None
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

    pose_sub = rospy.Subscriber('/orb_slam3/pose_out', PoseWithCovarianceStamped, pose_callback, queue_size=5)
    height_sub = rospy.Subscriber('/drone/height_estimate', Float64, height_callback, queue_size=5)
    orient_sub = rospy.Subscriber('/imu/9dof', Imu, orient_cb, queue_size=15)

    rospy.spin()
