#!/usr/bin/env python
import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import random
import numpy as np

pose_pub = None

# slam last values
lsy = None
lsyaw = 0

# imu last yaw
last_yaw = None

# init data
lsy_init = None
height_init = None
yaw_offs_init = 0

# ransac variables
ransac_pairs = []
ransac_complete = False
ransac_err = 1
ransac_m_scale = 0


def perform_ransac():
    global ransac_m_scale, ransac_err

    if len(ransac_pairs) < 40:
        return

    n_iter = 0
    while True:
        if n_iter > 5000:
            break

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

        n_iter = n_iter + 1


# noinspection PyUnresolvedReferences,PyTypeChecker
def height_callback(height):
    global lsy, height_init, lsy_init, ransac_complete
    if ransac_complete is False and lsy is not None and last_yaw is not None:
        if height_init is None:
            height_init = height.data
            lsy_init = lsy
        else:
            ransac_pairs.append((height.data - height_init, lsy - lsy_init))
            perform_ransac()

            if ransac_err < 1:
                print("SLAM RANSAC error: %f" % ransac_err)
            else:
                print("Ransac in progress. Frames: %d" % len(ransac_pairs))

            if ransac_err < 0.4:
                global yaw_offs_init
                print("RANSAC scale: %f su/m" % ransac_m_scale)

                # determine yaw offset
                yaw_offs_init = last_yaw - lsyaw
                yaw_offs_init = np.arctan2(np.sin(yaw_offs_init), np.cos(yaw_offs_init))
                print("Yaw offset: %f deg" % (yaw_offs_init * 180 / np.pi))

                ransac_complete = True

            lsy = None  # reset last slam y


def pose_callback(pose):
    if pose.pose.covariance[0] != 0:
        global lsy, lsyaw, lsy_init, height_init, yaw_offs_init, ransac_pairs, \
            ransac_complete, ransac_err, ransac_m_scale
        # reset estimator
        lsy = None
        lsyaw = 0
        lsy_init = None
        height_init = None
        yaw_offs_init = 0
        ransac_pairs = []
        ransac_complete = False
        ransac_err = 1
        ransac_m_scale = 0
        print("Scale estimator reset")
        return

    if ransac_complete is True:
        pose_absolute = PoseStamped()
        pose_absolute.header.stamp = rospy.get_rostime()
        lg_x = pose.pose.pose.position.x * ransac_m_scale
        lg_y = pose.pose.pose.position.z * ransac_m_scale

        pose_absolute.pose.position.y = np.cos(yaw_offs_init) * lg_x - np.sin(yaw_offs_init) * lg_y
        pose_absolute.pose.position.x = np.sin(yaw_offs_init) * lg_x + np.cos(yaw_offs_init) * lg_y
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

    pose_pub = rospy.Publisher('/orbslam3/pose_raw', PoseStamped, queue_size=1)

    pose_sub = rospy.Subscriber('/orb_slam3/pose_out', PoseWithCovarianceStamped, pose_callback, queue_size=1)
    height_sub = rospy.Subscriber('/drone/height_estimate', Float64, height_callback, queue_size=1)
    orient_sub = rospy.Subscriber('/imu/9dof', Imu, orient_cb, queue_size=15)

    rospy.spin()
