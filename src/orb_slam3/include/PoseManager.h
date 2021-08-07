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
    float height_last = 0;

public:
    bool datum_set = false;

    float yaw_mag_init = 0;
    float yaw_mag_curr = 0;

    explicit PoseManager(ros::NodeHandle *nh);

    void setGPSDatum(geometry_msgs::Quaternion orientation_last);

    void imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg);

    void gpsDataCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);

    void heightCallback(const std_msgs::Float64::ConstPtr &msg);

    void publishData(float px, float py);
};