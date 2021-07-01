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

#include <robot_localization/navsat_transform.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <chrono>
#include <thread>

#include "System.h"

#include "PoseManager.h"

using namespace std;

enum TrackerState {
    IDLE, WARMUP_COMPLETE, SLAM_TRACKING, DEAD_RECKONING
};

class Fusion {

private:
    float slam_ox = 0, slam_oy = 0;
    float px = 0, py = 0, p_yaw = 0;

    float yaw_offset = 0;

    Eigen::Quaternionf orientation;
    Eigen::Vector3f velocity;
    Eigen::Vector3f gravity;
    Eigen::Vector3f aBias;
    Eigen::Vector3f gBias;

    vector<sensor_msgs::ImuConstPtr> imuBuffer;
    std::mutex imuMutex;

    std::chrono::time_point<std::chrono::high_resolution_clock> time_last;

    PoseManager *poseManager;

    TrackerState state = IDLE;
    int warmupCounter = 0;

public:
    explicit Fusion(ros::NodeHandle *nh);

    void dataSLAM(ORB_SLAM3::System *mpSLAM, const cv::Mat &Tcw, vector<ORB_SLAM3::IMU::Point> &vImuMeas);

    void addIMUMeasurement(const sensor_msgs::ImuConstPtr &imu_msg);

    void deadReckoning(const vector<sensor_msgs::ImuConstPtr> &imuBuffer);

    void tracking();

    static void makeQuaternionFromVector(Eigen::Vector3f &inVec, Eigen::Quaternionf &outQuat);

};