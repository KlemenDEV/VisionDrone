#include <iostream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <opencv2/core/core.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "System.h"
#include "ImuTypes.h"

ORB_SLAM3::System *mpSLAM;
bool visualize;

ros::Publisher publisher;

void grabImage(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    Sophus::SE3f Twc = mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec()).inverse();
    Twc.normalize();

    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "slam";
    pose.header.stamp = cv_ptr->header.stamp;

    pose.pose.pose.position.x = (double) Twc.translation()(0);
    pose.pose.pose.position.y = (double) Twc.translation()(1);
    pose.pose.pose.position.z = (double) Twc.translation()(2);



    tf2::Matrix3x3 tf2_rot((double) Twc.rotationMatrix()(0, 0), (double) Twc.rotationMatrix()(0, 1), (double) Twc.rotationMatrix()(0, 2),
                           (double) Twc.rotationMatrix()(1, 0), (double) Twc.rotationMatrix()(1, 1), (double) Twc.rotationMatrix()(1, 2),
                           (double) Twc.rotationMatrix()(2, 0), (double) Twc.rotationMatrix()(2, 1), (double) Twc.rotationMatrix()(2, 2));
    tf2::Quaternion tf2_quat_rot;
    tf2_rot.getRotation(tf2_quat_rot);
    pose.pose.pose.orientation = tf2::toMsg(tf2_quat_rot);

    if (mpSLAM->mpTracker->mState == ORB_SLAM3::Tracking::OK) {
        pose.pose.covariance[0] = 0;
    } else {
        pose.pose.covariance[0] = 1;
    }

    publisher.publish(pose);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ORBSLAM3");
    ros::NodeHandle nh("~");

    nh.param<bool>("visualize", visualize, true);

    std::string path_to_vocabulary;
    nh.param<std::string>("path_to_vocabulary", path_to_vocabulary,
                          "/home/pylo/drone_ws/non_ros/orb_slam3/Vocabulary/ORBvoc.txt");
    std::string path_to_settings;
    nh.param<std::string>("path_to_settings", path_to_settings,
                          "/home/pylo/drone_ws/src/orb_slam3/orb_slam3.yaml");

    ORB_SLAM3::System SLAM(path_to_vocabulary, path_to_settings, ORB_SLAM3::System::MONOCULAR, visualize);
    mpSLAM = &SLAM;

    std::string topic_img;
    nh.param<std::string>("topic_img", topic_img, "/camera/infra1/image_rect_raw");
    ros::Subscriber sub_img = nh.subscribe(topic_img, 2, grabImage, ros::TransportHints().tcpNoDelay(true));

    publisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/orb_slam3/pose_out", 10);

    ros::spin();

    return 0;
}
