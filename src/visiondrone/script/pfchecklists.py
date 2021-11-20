#!/usr/bin/env python
import rospy
import rostopic

from ublox_msgs.msg import NavPVT7

num_sv = 0


def gps_cb(msg):
    global num_sv
    num_sv = msg.numSV


if __name__ == "__main__":
    rospy.init_node('pfchecklist')

    rospy.loginfo("Preflight checks started")

    topics = rospy.get_param("~topics")
    topics_list = [string.strip() for string in topics.splitlines()]
    active_topics = dict.fromkeys(topics_list, False)

    sub = rospy.Subscriber('/ublox/navpvt', NavPVT7, gps_cb, queue_size=10)

    r = rostopic.ROSTopicHz(-1)

    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        for topic in topics_list:
            rate = r.get_hz(topic)
            if rate is not None and rate > 0:
                active_topics[topic] = True

        if all(active_topics.values()):
            rospy.loginfo("======================================")
            rospy.loginfo(">>> GPS SV number: %d <<<" % num_sv)
            rospy.loginfo(">>> PFC DONE, ALL TOPICS ARE READY <<<")
            rospy.loginfo("======================================")
            rospy.signal_shutdown("Preflight checklist done")

        rate.sleep()
