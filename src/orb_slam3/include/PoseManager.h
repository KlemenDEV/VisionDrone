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

#include <orb_slam3/SetDatum.h>

#include <chrono>
#include <thread>

enum FlightState {
    REST,
    TAKEOFF,
    FRONT_FLIGHT,
    FREE_FLIGHT
};

using namespace std;

class PoseManager {

private:
    ros::Publisher point_pub;
    ros::Publisher gps_pub;

    ros::ServiceClient set_datum_client;

    ros::Subscriber imuorient_sub;
    int imuDataCount = 0;

    ros::Subscriber gpsfix_sub;

    sensor_msgs::NavSatFix::ConstPtr gps_last;

    ros::Subscriber height_sub;

    geometry_msgs::Quaternion orientation_last;

    FlightState flightState = REST;

    std::chrono::time_point<std::chrono::high_resolution_clock> time_last;
    float opx = 0, opy = 0;

    double yaw_diff = 0;
    double yaw_mag_avg = 0;
    int yaw_diff_count = 0;

    float offx = 0, offy = 0;

public:
    bool datum_set = false;

    float yaw_mag_init = 0;
    float yaw_mag_curr = 0;

    float height_last = 0;

    explicit PoseManager(ros::NodeHandle *nh);

    void setGPSDatum(float px, float py);

    void imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg);

    void gpsDataCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);

    void heightCallback(const std_msgs::Float64::ConstPtr &msg);

    void publishData(float px, float py);
};