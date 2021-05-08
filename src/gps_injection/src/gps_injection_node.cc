#include <ros/ros.h>

#include <ublox_msgs/NavPOSLLH.h>
#include <ublox_msgs/NavVELNED.h>
#include <ublox_msgs/NavSOL.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <geographic_msgs/GeoPose.h>
#include <geographic_msgs/GeoPoint.h>

#include <mavros_msgs/RCIn.h>

#include <robot_localization/navsat_transform.h>

#include "UBXSender.h"
#include "GPSData.h"

// holds the current position as sent from the GPS module/antenna
GPSData gps_pos_antenna;

// holds the current position predicted from the visual system
GPSData gps_pos_predicted;

// if true, predicted position is injected instead of GPS pass-through
volatile bool inject_data = false;

// if true, home position has been set (happens when 3D fix is obtained)
bool home_set = false;

ros::Subscriber imuorient_sub;
geometry_msgs::Quaternion orientation_last;

void rcInCallback(const mavros_msgs::RCIn::ConstPtr &msg) {
    // switch depending on the Channel 8 value (> 1500 -> inject data)
    bool set_value = msg->channels[7] > 1800;

    // detect if value changed
    if (set_value ^ inject_data) {
        inject_data = set_value;
        if (inject_data) {
            ROS_INFO("GPS injection system source: VISUAL PREDICTION");
        } else {
            ROS_INFO("GPS injection system source: UBLOX ANTENNA");
        }
    }
}

void imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    orientation_last = msg->orientation;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_injection_node");
    ros::NodeHandle nh;

    ros::Rate loop_rate(50); // GPS has 4/5Hz, use 10Hz to fulfill Shannon's requirement

    // UBLOX antenna/module subscribers
    ros::Subscriber navposllh_sub = nh.subscribe<ublox_msgs::NavPOSLLH>
            ("/ublox/navposllh", 1, &GPSData::consume, &gps_pos_antenna);
    ros::Subscriber navvelned_sub = nh.subscribe<ublox_msgs::NavVELNED>
            ("/ublox/navvelned", 1, &GPSData::consume, &gps_pos_antenna);
    ros::Subscriber navsol_sub = nh.subscribe<ublox_msgs::NavSOL>
            ("/ublox/navsol", 1, &GPSData::consume, &gps_pos_antenna);

    // Position prediction subscribers
    ros::Subscriber predictpose_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("/visionpos/pose", 1, &GPSData::consume, &gps_pos_predicted);
    ros::Subscriber predictvel_sub = nh.subscribe<geometry_msgs::TwistWithCovarianceStamped>
            ("/visionpos/velocity", 1, &GPSData::consume, &gps_pos_predicted);

    // IMU orientation (for initial pose)
    imuorient_sub = nh.subscribe<sensor_msgs::Imu>
            ("/mavros/imu/data", 1, imuDataCallback);

    // Set inject_data depending on mavlink RC messages
    ros::Subscriber mavlinkrc_sub = nh.subscribe<mavros_msgs::RCIn>
            ("/mavros/rc/in", 1, rcInCallback);

    // Set datum service client
    ros::ServiceClient set_datum_client = nh.serviceClient<robot_localization::SetDatum>("set_datum");

    UBXSender ubxSender;
    uint32_t gps_time = 0;

    ROS_DEBUG("GPS injector ready");

    while (ros::ok()) {
        if (inject_data && gps_pos_predicted.isComplete()) {
            gps_pos_predicted.setTime(gps_time); // use local time

            ubxSender.sendData(&gps_pos_predicted);

            gps_pos_predicted.markDirty(); // mark data dirty so we do not re-send same data
        } else if (!inject_data &&
                   gps_pos_antenna.isComplete()) { // if we are not in injection mode, send data if complete
            gps_pos_antenna.setTime(gps_time); // use local time

            ubxSender.sendData(&gps_pos_antenna);

            gps_pos_antenna.markDirty(); // mark data dirty so we do not re-send same data
        }

        if (!home_set && gps_pos_antenna.isComplete() && gps_pos_antenna.has3DLock()) {
            geographic_msgs::GeoPoint datumPosition;
            datumPosition.latitude = gps_pos_antenna.latitude * 1e-7; // deg
            datumPosition.longitude = gps_pos_antenna.longitude * 1e-7; // deg
            datumPosition.altitude = gps_pos_antenna.altitude_msl * 1e-3; // m

            geographic_msgs::GeoPose datumPose;
            datumPose.position = datumPosition;
            datumPose.orientation = orientation_last;

            robot_localization::SetDatum setDatum;
            setDatum.request.geo_pose = datumPose;

            if (set_datum_client.call(setDatum)) {
                ROS_INFO("GLOBAL GPS HOME LOCATION (DATUM) set and propagated to other services");
                ROS_INFO("DATUM: lat: %f, lon: %f, alt: %f", datumPosition.latitude, datumPosition.longitude,
                         datumPosition.altitude);
            } else {
                ROS_ERROR("FAILED TO PROPAGATE GLOBAL GPS HOME LOCATION (DATUM)");
            }

            // we no longer need imu data, unsubscribe
            imuorient_sub.shutdown();

            home_set = true;
        }

        // increment GPS time for 20ms (50Hz loop)
        gps_time += 20;

        ros::spinOnce();

        loop_rate.sleep();
    }

    ubxSender.closeSerialPort();

    ros::shutdown();

    return 0;
}
