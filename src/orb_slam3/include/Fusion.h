#pragma once

#include <opencv2/core/core.hpp>

#include<ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <nav_msgs/Odometry.h>

using namespace std;

class Fusion {

private:
    ros::Publisher point_pub;

public:
    explicit Fusion(ros::NodeHandle *nh);

    void dataSLAM(const cv::Mat& Tcw);

};