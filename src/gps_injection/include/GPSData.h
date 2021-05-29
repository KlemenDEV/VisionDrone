#pragma once

#include <math.h>

#include <ublox_msgs/NavPVT7.h>

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include "ubx.h"

class GPSData {

public:
    uint32_t time;
    uint16_t week;

    int32_t longitude;
    int32_t latitude;
    int32_t altitude_msl;

    int32_t ned_north;
    int32_t ned_east;
    int32_t ned_down;
    uint32_t speed_2d;
    int32_t heading_2d;

    uint8_t fix_type;
    uint8_t fix_status;
    uint16_t position_DOP;
    uint8_t satellites;

    bool hasPOSLLH;
    bool hasSOLUTION;
    bool hasVELNED;

public:
    bool isComplete();
    bool has3DLock();

    void setTime(uint32_t time);
    void markDirty();

    // Consumer to assemble GPSData from UBLOX messages
    void consume(const ublox_msgs::NavPVT7::ConstPtr &msg);

    // Set of 2 consumers to assemble GPSData from location predictions
    void consume(const sensor_msgs::NavSatFix::ConstPtr &msg);
    void consume(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg);

    ubx_nav_posllh getPOSLLH();
    ubx_nav_status getSTATUS();
    ubx_nav_solution getSOLUTION();
    ubx_nav_velned getVELNED();

};