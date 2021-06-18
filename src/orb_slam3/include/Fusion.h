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

#include "System.h"

using namespace std;

class Fusion {

private:
    ros::Publisher point_pub;
    ros::Publisher point_pub2;

    float ox = 0, oy = 0, oz = 0;
    float odx = 0, ody = 0, odz = 0;

    float px = 0, py = 0, pz = 0;

    bool reset = true;
    bool tracking_started = false;

    ros::ServiceClient set_datum_client;

    ros::Subscriber imuorient_sub;
    geometry_msgs::Quaternion orientation_last;

    ros::Subscriber gpsfix_sub;
    sensor_msgs::NavSatFix::ConstPtr gps_last;

    ros::Subscriber height_sub;
    float height_last = 0;

    float yaw_corr = 0;
    int avgcounter = 0;

public:
    explicit Fusion(ros::NodeHandle *nh);

    void dataSLAM(ORB_SLAM3::System *mpSLAM, const cv::Mat &Tcw);

    void setGPSDatum();

    void imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg);

    void gpsDataCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);

    void heightCallback(const std_msgs::Float64::ConstPtr &msg);

};