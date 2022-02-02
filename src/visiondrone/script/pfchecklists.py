#!/usr/bin/env python
import rospy
import rostopic

from ublox_msgs.msg import NavPVT

num_sv = 0


def gps_cb(msg):
    global num_sv
    num_sv = msg.numSV


def topic_cb(_, topic):
    active_topics[topic] = True


if __name__ == "__main__":
    rospy.init_node('pfchecklist')
    topics = rospy.get_param("~topics").replace('\n', ' ').replace('\r', ' ')
    topics_list = topics.split()
    active_topics = dict.fromkeys(topics_list, False)

    sub = rospy.Subscriber('/ublox/navpvt', NavPVT, gps_cb, queue_size=1)

    for topic in topics_list:
        rospy.Subscriber(topic, rospy.AnyMsg, topic_cb, queue_size=1, callback_args=topic)

    ros_rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        if all(active_topics.values()):
            rospy.logfatal("######################################")
            rospy.logfatal("#       GPS SV number: %02d            #" % num_sv)
            rospy.logfatal("#   PFC DONE, ALL TOPICS ARE READY   #")
            rospy.logfatal("######################################")
            rospy.signal_shutdown("Preflight checklist done")

        ros_rate.sleep()
