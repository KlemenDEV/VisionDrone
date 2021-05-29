#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomToPath:
    def __init__(self, source, dest):
        self.path_pub = rospy.Publisher(dest, Path, latch=True, queue_size=10)
        self.odom_sub = rospy.Subscriber(source, Odometry, self.odom_cb, queue_size=10)
        self.path = Path()

    def odom_cb(self, msg):
        cur_pose = PoseStamped()
        cur_pose.header = msg.header
        cur_pose.pose = msg.pose.pose
        self.path.header = msg.header
        self.path.poses.append(cur_pose)
        self.path_pub.publish(self.path)


if __name__ == '__main__':
    rospy.init_node('pathgen')
    odom_to_path = OdomToPath(rospy.get_param("~odom"), rospy.get_param("~path"))
    rospy.spin()
