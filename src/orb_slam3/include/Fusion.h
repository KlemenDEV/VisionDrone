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

using namespace std;

class Fusion {

private:
    ros::Publisher point_pub;
    ros::Publisher gps_pub;

    // old state data
    float slam_ox = 0, slam_oy = 0;
    float odx = 0, ody = 0;

    // states
    float px = 0, py = 0, pz = 0;
    Eigen::Quaternionf orientation;
    Eigen::Vector3f velocity;

    Eigen::Vector3f gravity;
    Eigen::Vector3f aBias;
    Eigen::Vector3f gBias;

    bool tracking_started = false;

    bool dead_reckoning = false;

    ros::ServiceClient set_datum_client;

    ros::Subscriber imuorient_sub;
    geometry_msgs::Quaternion orientation_last;

    ros::Subscriber gpsfix_sub;
    sensor_msgs::NavSatFix::ConstPtr gps_last;

    ros::Subscriber height_sub;
    float height_last = 0;
    float height_pre_last = 0;

    float ochg;

    float yaw_init_mag;
    float yaw_init_slam;

    queue<sensor_msgs::ImuConstPtr> imuBuffer;
    std::mutex imuMutex;

    std::chrono::time_point<std::chrono::high_resolution_clock> time_last;

    std::thread tracker;

public:
    explicit Fusion(ros::NodeHandle *nh);

    static void makeQuaternionFromVector(Eigen::Vector3f &inVec, Eigen::Quaternionf &outQuat);

    void dataSLAM(ORB_SLAM3::System *mpSLAM, const cv::Mat &Tcw, vector<ORB_SLAM3::IMU::Point> &vImuMeas);

    void deadReckoning(const vector<ORB_SLAM3::IMU::Point> &vImuMeas);

    void setGPSDatum();

    void imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg);

    void gpsDataCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);

    void heightCallback(const std_msgs::Float64::ConstPtr &msg);

    void publishData();

    void tracking();

    void addIMUMeasurement(const sensor_msgs::ImuConstPtr &imu_msg);
};