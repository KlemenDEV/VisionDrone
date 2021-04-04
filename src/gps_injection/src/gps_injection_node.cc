#include <ros/ros.h>

#include <ublox_msgs/NavPOSLLH.h>
#include <ublox_msgs/NavVELNED.h>
#include <ublox_msgs/NavSOL.h>

#include "UBXSender.h"
#include "GPSData.h"

// holds point where GPS is initiated (when 3D lock is obtained)
GPSData *gps_pos_origin = nullptr;

// hold the current position as sent from the GPS module/antenna
GPSData gps_pos_antenna;

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_injection_node");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10); // GPS has 4/5Hz, use 10Hz to fulfill Shannon's requirement

    ros::Subscriber navposllh_sub = nh.subscribe<ublox_msgs::NavPOSLLH>
            ("/ublox/navposllh", 1, &GPSData::consume, &gps_pos_antenna);
    ros::Subscriber navvelned_sub = nh.subscribe<ublox_msgs::NavVELNED>
            ("/ublox/navvelned", 1, &GPSData::consume, &gps_pos_antenna);
    ros::Subscriber navsol_sub = nh.subscribe<ublox_msgs::NavSOL>
            ("/ublox/navsol", 1, &GPSData::consume, &gps_pos_antenna);

    UBXSender ubxSender;
    uint32_t gps_time = 0;

    bool inject_data = false;

    ROS_DEBUG("GPS injector ready");

    while (ros::ok()) {
        // if gps origin is not set yet, wait for it
        if (gps_pos_origin == nullptr) {
            // check if we got any data from the GPS module
            if (gps_pos_antenna.isComplete() && gps_pos_antenna.has3DLock()) {
                gps_pos_origin = new GPSData(gps_pos_antenna); // origin location is placed on Heap

                ROS_INFO("GPS origin set");
                gps_pos_origin->printData(); // Print location for debugging
            } else {
                ROS_INFO_THROTTLE(5, "GPS origin not ready yet!");
            }
        }

        // TODO: set inject_data depending on mavlink RC messages

        if (inject_data && gps_pos_origin != nullptr) { // TODO: additional check if data is reliable and relevant
            // TODO: read data from topic and send data offset to true GPS
            // TODO.setTime(gps_time); // use local time
            // ubxSender.sendData(TODO);
        } else if (gps_pos_antenna.isComplete()) { // if we are not in injection mode, send data if complete
            gps_pos_antenna.setTime(gps_time); // use local time

            ubxSender.sendData(&gps_pos_antenna);

            gps_pos_antenna.markDirty(); // mark data dirty so we do not re-send same data
        }

        // increment GPS time for 100ms (10Hz loop)
        gps_time += 100;

        ros::spinOnce();

        loop_rate.sleep();
    }

    ubxSender.closeSerialPort();

    ros::shutdown();

    return 0;
}
