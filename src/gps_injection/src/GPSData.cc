#include "GPSData.h"

void GPSData::consume(const uublox_msgs::NavPOSLLH::ConstPtr &msg) {
    if (this->isComplete()) // if we consume on complete data, we re-initialize the data
        this->markDirty();

    // TODO: consume

    this->hasPOSLLH = true;
}

void GPSData::consume(const uublox_msgs::NavVELNED::ConstPtr &msg) {
    if (this->isComplete()) // if we consume on complete data, we re-initialize the data
        this->markDirty();

    // TODO: consume

    this->hasVELNED = true;
}

void GPSData::consume(const uublox_msgs::NavSOL::ConstPtr &msg) {
    if (this->isComplete()) // if we consume on complete data, we re-initialize the data
        this->markDirty();

    // TODO: consume

    this->hasSOLUTION = true;
}

bool GPSData::isComplete() {
    return this->hasPOSLLH && this->hasSOLUTION && this->hasVELNED;
}

void GPSData::markDirty() {
    this->hasPOSLLH = this->hasSOLUTION = this->hasVELNED = false;
}

bool GPSData::has3DLock() {
    return false; // TODO: implement this
}

void GPSData::setTime(uint32_t time) {
    this->time = time;
}

ubx_nav_posllh GPSData::getPOSLLH() {
    ubx_nav_posllh msg_posllh;
    msg_posllh.time = gps_time;
    msg_posllh.longitude = 0;
    msg_posllh.latitude = 0;
    msg_posllh.altitude_msl = 0;
    return msg_posllh;
}

ubx_nav_status GPSData::getSTATUS() {
    ubx_nav_status msg_status;
    msg_status.time = gps_time;
    msg_status.fix_type = 0x03;
    msg_status.fix_status = 0b1101;
    return msg_status;
}

ubx_nav_solution GPSData::getSOLUTION() {
    ubx_nav_solution msg_solution;
    msg_solution.time = gps_time;
    msg_solution.week = 1721;
    msg_solution.fix_type = 0x03;
    msg_solution.fix_status = 0b1101;
    msg_solution.position_DOP = 5;
    msg_solution.satellites = 5;
    return msg_solution;
}

ubx_nav_velned GPSData::getVELNED() {
    ubx_nav_velned msg_velned;
    msg_velned.time = gps_time;
    msg_velned.ned_north = 0;
    msg_velned.ned_east = 0;
    msg_velned.ned_down = 0;
    msg_velned.speed_2d = 0;
    msg_velned.heading_2d = 0;
    return msg_velned;
}
