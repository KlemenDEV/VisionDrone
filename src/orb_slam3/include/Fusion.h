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

#include "PoseManager.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;

enum TrackerState {
    IDLE, SLAM_TRACKING, DEAD_RECKONING
};

class Fusion {

private:
    float slam_ox = 0, slam_oy = 0;
    float px = 0, py = 0, p_yaw = 0;

    float yaw_offset = 0;

    Eigen::Vector3d velocity;
    Eigen::Vector3d aBias;

    vector<sensor_msgs::Imu> imuBuffer;
    std::mutex imuMutex;

    std::chrono::time_point<std::chrono::high_resolution_clock> time_last;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::Imu> syncPolicy;
    typedef message_filters::Synchronizer <syncPolicy> syncer;
    message_filters::Subscriber <sensor_msgs::Imu> imu1;
    message_filters::Subscriber <sensor_msgs::Imu> imu2;
    std::shared_ptr<syncer> syncptr;

    PoseManager *poseManager;

    TrackerState state = IDLE;

public:
    explicit Fusion(ros::NodeHandle *nh);

    void dataSLAM(ORB_SLAM3::System *mpSLAM, const cv::Mat &Tcw, vector<ORB_SLAM3::IMU::Point> &vImuMeas);

    void deadReckoning();

    void tracking();

    void imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg, const sensor_msgs::Imu::ConstPtr &msg2);

};