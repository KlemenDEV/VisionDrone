#!/usr/bin/env python
import numpy as np
import rospy
import tf.transformations
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64
from velocity_integrator.srv import SetDatum
import time
import math

TWCS = TwistWithCovarianceStamped

datum = None
last_odom_msg = None
last_vel_msg = None
last_yaw = None
height = None

# Earth's radius, sphere
R = 6378137


def height_cb(msg):
    global height
    height = msg.data


def imu_cb(msg):
    global last_yaw
    (_, _, last_yaw) = tf.transformations.euler_from_quaternion([
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w,
    ])


def odom_cb(msg):
    global last_odom_msg
    last_odom_msg = msg


def velocity_cb(msg):
    global last_vel_msg
    last_vel_msg = msg


def handle_datum(req):
    global datum
    datum = np.array([req.geo_pose.position.latitude, req.geo_pose.position.longitude, req.geo_pose.position.altitude])
    return []


if __name__ == "__main__":
    rospy.init_node('local2gps')

    odom_sub = rospy.Subscriber(rospy.get_param("~pose"), PoseStamped, odom_cb, queue_size=50)
    velocity_sub = rospy.Subscriber(rospy.get_param("~velocity"), TWCS, velocity_cb, queue_size=50)

    imu_sub = rospy.Subscriber('/imu/9dof', Imu, imu_cb, queue_size=50)
    height_sub = rospy.Subscriber('/drone/height_estimate', Float64, height_cb, queue_size=50)

    gps_pose_pub = rospy.Publisher('/estimate/pose', NavSatFix, queue_size=50)
    gps_velocity_pub = rospy.Publisher('/estimate/velocity', TWCS, queue_size=50)

    rospy.Service('datum_sim', SetDatum, handle_datum)

    r = rospy.Rate(5)  # 5 Hz

    altitude_prev = None
    t_last = 0

    while not rospy.is_shutdown():
        gps_pose = NavSatFix()
        gps_velocity = TWCS()

        if last_vel_msg is not None and last_odom_msg is not None \
                and datum is not None and last_yaw is not None and height is not None:
            if altitude_prev is None:
                altitude_prev = height
                t_last = time.time()
                continue

            # http://www.edwilliams.org/avform147.htm#LL
            # https://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
            dn = last_odom_msg.pose.position.x
            de = last_odom_msg.pose.position.y
            dLat = -dn / R
            dLon = de / (R * math.cos(math.pi * datum[0] / 180))
            gps_pose.latitude = datum[0] + dLat * 180 / math.pi  # deg
            gps_pose.longitude = datum[1] + dLon * 180 / math.pi  # deg

            gps_velocity.twist.twist.linear.x = \
                + last_vel_msg.twist.twist.linear.x * math.cos(-last_yaw) \
                - last_vel_msg.twist.twist.linear.y * math.sin(-last_yaw)  # east, m/s
            gps_velocity.twist.twist.linear.y = \
                - last_vel_msg.twist.twist.linear.x * math.sin(-last_yaw) \
                - last_vel_msg.twist.twist.linear.y * math.cos(-last_yaw)  # north, m/s

            gps_pose.altitude = datum[2] + height  # m
            gps_velocity.twist.twist.linear.z = (altitude_prev - height) / (time.time() - t_last)  # down, m/s

            gps_pose_pub.publish(gps_pose)
            gps_velocity_pub.publish(gps_velocity)

            t_last = time.time()

        r.sleep()
