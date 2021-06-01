#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class OdomToPath:
    def __init__(self, source, dest):
        self.odom_sub = rospy.Subscriber(source, PoseStamped, self.odom_cb, queue_size=10)
        self.path = Path()

    def odom_cb(self, msg):
        msg.pose


if __name__ == '__main__':
    rospy.init_node('posetomatlab')
    odom_to_path = OdomToPath(rospy.get_param("~pose"), rospy.get_param("~path"))
    rospy.spin()
