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

#include <velocity_integrator/SetDatum.h>

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
    ros::ServiceClient set_datum_client_ref;

    ros::Subscriber imuorient_sub;
    ros::Subscriber gpsfix_sub;
    ros::Subscriber height_sub;

    sensor_msgs::NavSatFix::ConstPtr gps_last;
    geometry_msgs::Quaternion orientation_last;

    FlightState flightState = REST;

    volatile bool datum_set = false;

    volatile double yaw_diff = 0;
    volatile double yaw_mag_avg = 0;
    volatile int yaw_diff_count = 0;

public:
    volatile float yaw_mag_curr = 0;
    volatile float height_last = 0;

    explicit PoseManager(ros::NodeHandle *nh);

    void publishData(float px, float py, float vx, float vy);

    void setGPSDatum(float px, float py);

    void imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg);

    void gpsDataCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);

    void heightCallback(const std_msgs::Float64::ConstPtr &msg);
};