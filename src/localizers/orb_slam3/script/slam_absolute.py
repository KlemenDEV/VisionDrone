#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Float64
import random

pose_pub = None

height_init = None
lsy = None

ransac_pairs = []
ransac_complete = False
ransac_err = 1
ransac_m_scale = 0


def perform_ransac():
    global ransac_m_scale, ransac_err

    if len(ransac_pairs) < 25:
        return

    n_iter = 0
    while True:
        if n_iter > 5000:
            break

        r_gt_y, r_sp_y = random.choice(ransac_pairs)
        new_ransac_m_scale = r_gt_y / r_sp_y

        err_curr = 0
        for gt_y, sp_y in ransac_pairs:
            if abs(gt_y - sp_y * new_ransac_m_scale) > 1:
                err_curr = err_curr + 1

        err_curr /= float(len(ransac_pairs))

        if err_curr < ransac_err:
            ransac_m_scale = new_ransac_m_scale
            ransac_err = err_curr

        n_iter = n_iter + 1


def height_callback(height):
    global lsy, height_init, ransac_complete
    if ransac_complete is False and lsy is not None:
        if height_init is None:
            height_init = height.data + lsy
        else:
            ransac_pairs.append((height.data - height_init, lsy))
            perform_ransac()
            print("SLAM RANSAC error: %f" % ransac_err)

            if ransac_err <= 0.3:
                print("Ransac scale: %f" % ransac_m_scale)
                ransac_complete = True

            lsy = None  # reset last slam y


def pose_callback(pose):
    global lsy

    if pose.pose.covariance[0] != 0:
        return

    lsy = -pose.pose.pose.position.y

    if ransac_complete is True:
        pose_absolute = PoseStamped()
        pose_absolute.header.stamp = rospy.get_rostime()
        pose_absolute.pose.position.x = pose.pose.pose.position.x * ransac_m_scale
        pose_absolute.pose.position.y = pose.pose.pose.position.z * ransac_m_scale
        pose_pub.publish(pose_absolute)


if __name__ == "__main__":
    rospy.init_node('simulator')

    pose_pub = rospy.Publisher('/orbslam3/pose_raw', PoseStamped, queue_size=1)

    pose_sub = rospy.Subscriber('/orb_slam3/pose_out', PoseWithCovarianceStamped, pose_callback, queue_size=1)
    height_sub = rospy.Subscriber('/drone/height_estimate', Float64, height_callback, queue_size=1)

    rospy.spin()
