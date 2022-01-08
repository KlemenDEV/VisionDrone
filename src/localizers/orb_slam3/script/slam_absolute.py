#!/usr/bin/env python
import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import numpy as np
import random


class LPF:
    def __init__(self, window_size):
        self.window_size = window_size
        self.values = []
        self.sum = 0

    def filter(self, value):
        self.values.append(value)
        self.sum += value
        if len(self.values) > self.window_size:
            self.sum -= self.values.pop(0)
        return float(self.sum) / len(self.values)


pose_pub = None
vel_pub = None
vel_enu_pub = None
p_off_x = None
p_off_y = None
p_off_z = None

# slam old values
lsy = None
lsx = None
lsz = None
lst = None

# orientation correction
last_yaw = None
orient_slam = None
R_sw = 0

# ransac variables
ransac_pairs = []
ransac_complete = False
ransac_err = 1
ransac_m_k = 0
ransac_m_n = 0

f_enu_x = LPF(15)
f_enu_y = LPF(15)


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
    if ransac_complete is False and lsy is not None and orient_slam is not None:
        ransac_pairs.append((height.data, lsy))

        if len(ransac_pairs) < 60:
            print("Collecting data for ransac. Frames: %d" % len(ransac_pairs))
        else:
            perform_ransac(0.4, 5000)
            print("SLAM RANSAC error: %f" % ransac_err)

        if ransac_err <= 0.15:
            print("RANSAC scale: %f su/m" % ransac_m_k)

            global R_sw
            R_sw = tf.transformations.quaternion_inverse(orient_slam)

            ransac_complete = True

        lsy = None  # reset last slam y


def pose_callback(pose):
    global ransac_pairs, orient_slam, R_sw, p_off_x, p_off_y, p_off_z, \
        ransac_complete, ransac_err, ransac_m_k, lsx, lsz, lst, lsy

    # reset estimator if invalid data
    if pose.pose.covariance[0] != 0:
        lsy = None
        orient_slam = None
        lsx = None
        lsz = None
        lst = None
        p_off_x = None
        p_off_y = None
        p_off_z = None
        R_sw = None
        ransac_pairs = []
        ransac_complete = False
        ransac_err = 1
        ransac_m_k = 0
        return

    if ransac_complete is True:
        lg_x = pose.pose.pose.position.x * ransac_m_k * 1.5
        lg_y = pose.pose.pose.position.y * ransac_m_k * 1.5
        lg_z = pose.pose.pose.position.z * ransac_m_k * 1.5

        if p_off_x is None or p_off_y is None:
            p_off_x = lg_x
            p_off_y = lg_y
            p_off_z = lg_z

        lg_x = lg_x - p_off_x
        lg_y = lg_y - p_off_y
        lg_z = lg_z - p_off_z

        lg = [lg_x, lg_y, lg_z, 0]
        lg_rotated = tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(R_sw, lg),
                                                            tf.transformations.quaternion_conjugate(R_sw))

        pose_absolute = PoseStamped()
        pose_absolute.header.stamp = rospy.get_rostime()
        pose_absolute.pose.position.x = lg_rotated[2]
        pose_absolute.pose.position.y = -lg_rotated[0]
        pose_absolute.pose.position.z = lg_rotated[1]
        pose_pub.publish(pose_absolute)

        if lsz is not None:
            dt = pose.header.stamp.to_sec() - lst
            if dt > 0:
                vx_enu = f_enu_x.filter((pose_absolute.pose.position.y - lsz) / dt)
                vy_enu = -f_enu_y.filter((pose_absolute.pose.position.x - lsx) / dt)
                vel_enu = TwistWithCovarianceStamped()
                vel_enu.header.stamp = rospy.get_rostime()
                vel_enu.header.frame_id = "uav_velocity_enu"
                vel_enu.twist.twist.linear.x = vx_enu
                vel_enu.twist.twist.linear.y = vy_enu
                vel_enu.twist.twist.linear.z = 0
                vel_enu_pub.publish(vel_enu)

                vel = TwistWithCovarianceStamped()
                vel.header.stamp = rospy.get_rostime()
                vel.header.frame_id = "uav_velocity"
                vel.twist.twist.linear.x = (np.cos(-last_yaw) * vx_enu - np.sin(-last_yaw) * vy_enu)
                vel.twist.twist.linear.y = (np.sin(-last_yaw) * vx_enu + np.cos(-last_yaw) * vy_enu)
                vel.twist.twist.linear.z = 0
                vel_pub.publish(vel)

        lsx = pose_absolute.pose.position.x
        lsz = pose_absolute.pose.position.y
        lst = pose.header.stamp.to_sec()
    else:
        lsy = -pose.pose.pose.position.y
        orient_slam = [
            pose.pose.pose.orientation.x,
            pose.pose.pose.orientation.y,
            pose.pose.pose.orientation.z,
            pose.pose.pose.orientation.w
        ]


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
