#pragma once

#include <opencv2/core/core.hpp>

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <std_msgs/Float64.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geographic_msgs/GeoPose.h>
#include <geographic_msgs/GeoPoint.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>

#include <chrono>
#include <thread>

#include "System.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;

class Fusion {

private:
    float slam_ox = 0, slam_oy = 0;
    double t_last = -1;

    ros::Publisher publisher_velocity;

public:
    explicit Fusion(ros::NodeHandle *nh);

    void dataSLAM(ORB_SLAM3::System *mpSLAM, const cv::Mat &Tcw);

};