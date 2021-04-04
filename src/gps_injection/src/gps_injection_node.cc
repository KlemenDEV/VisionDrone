#include <ros/ros.h>

#include <ublox_msgs/NavPOSLLH.h>
#include <ublox_msgs/NavVELNED.h>
#include <ublox_msgs/NavSOL.h>

#include "UBXSender.h"
#include "GPSData.h"

// holds point where GPS is initiated (when 3D lock is obtained)
GPSData *gps_pos_origin = nullptr;

// holds the current position as sent from the GPS module/antenna
GPSData gps_pos_antenna;

// holds the current position predicted from the visual system
GPSData gps_pos_predicted;

// if true, predicted position is injected instead of GPS pass-through
bool inject_data = false;

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

    // TODO: set inject_data depending on mavlink RC messages

    // TODO: this should be called in callback from augmented position topic
    // if (gps_pos_origin != nullptr)
    //     gps_pos_predicted.withCartesian(0, 0, 0, 0, 0, 0, 0);
    // additionally it should be checked if data is not too far from GPS reported position
    // and if this is the case, data should be marked dirty so it is not used and injection is skipped

    UBXSender ubxSender;
    uint32_t gps_time = 0;

    ROS_DEBUG("GPS injector ready");

    while (ros::ok()) {
        // if gps origin is not set yet, try to set it
        if (gps_pos_origin == nullptr) {
            // check if we got any data from the GPS module
            if (gps_pos_antenna.isComplete() && gps_pos_antenna.has3DLock()) {
                gps_pos_origin = new GPSData(gps_pos_antenna); // origin location is placed on Heap

                ROS_INFO("GPS origin set");
                gps_pos_origin->printData(); // Print location to ros log for debugging
            } else {
                ROS_INFO_THROTTLE(5, "GPS origin not ready yet!");
            }
        }

        if (inject_data && gps_pos_predicted.isComplete()) {
            gps_pos_predicted.setTime(gps_time); // use local time

            ubxSender.sendData(&gps_pos_predicted);

            gps_pos_predicted.markDirty(); // mark data dirty so we do not re-send same data
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
