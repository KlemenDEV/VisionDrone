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

void navPosllhCallback(const uublox_msgs::NavPOSLLH::ConstPtr &msg) {
    gps_pos_antenna.consume(msg);
}

void navVelnedCallback(const uublox_msgs::NavVELNED::ConstPtr &msg) {
    gps_pos_antenna.consume(msg);
}

void navSolCallback(const uublox_msgs::NavSOL::ConstPtr &msg) {
    gps_pos_antenna.consume(msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "GPSGenerator");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10); // GPS has 4/5Hz, use 10Hz to fulfill Shannon's requirement

    nh.subscribe("ublox/nav/posllh", 1, navPosllhCallback);
    nh.subscribe("ublox/nav/velned", 1, navVelnedCallback);
    nh.subscribe("ublox/nav/sol", 1, navSolCallback);

    UBXSender ubxSender;
    uint32_t gps_time = 0;

    bool inject_data = false;

    while (ros::ok()) {
        // if gps origin is not set yet, wait for it
        if (gps_pos_origin == nullptr) {
            // check if we got any data from the GPS module
            if (gps_pos_antenna.isComplete() && gps_pos_antenna.has3DLock()) {
                gps_pos_origin = new GPSData(gps_pos_antenna); // origin location is placed on Heap
                ROS_INFO("GPS origin set");
                gps_pos_origin->printData(); // Print location for debugging
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

        // increment GPS time for 200ms (5Hz loop)
        gps_time += 200;

        ros::spinOnce();
        loop_rate.sleep();
    }

    ubxSender.close();

    ros::shutdown();

    return 0;
}
