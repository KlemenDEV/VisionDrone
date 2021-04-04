#pragma once

#include <ublox_msgs/NavPOSLLH.h>
#include <ublox_msgs/NavVELNED.h>
#include <ublox_msgs/NavSOL.h>

#include "ubx.h"

class GPSData {

public:
    uint32_t time;
    uint16_t week = 1721;

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

    void consume(const uublox_msgs::NavPOSLLH::ConstPtr &msg);
    void consume(const uublox_msgs::NavVELNED::ConstPtr &msg);
    void consume(const uublox_msgs::NavSOL::ConstPtr &msg);

    void printData();

    ubx_nav_posllh getPOSLLH();
    ubx_nav_status getSTATUS();
    ubx_nav_solution getSOLUTION();
    ubx_nav_velned getVELNED();
};