#include "GPSData.h"

void GPSData::consume(const uublox_msgs::NavPOSLLH::ConstPtr &msg) {
    if (this->isComplete()) // if we consume on complete data, we re-initialize the data
        this->markDirty();

    // https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/NavPOSLLH.msg
    this->longitude = msg->lon; // [deg / 1e-7]
    this->latitude = msg->lat; // [deg / 1e-7]
    this->altitude_msl = msg->hMSL; // Height above mean sea level [mm]

    this->hasPOSLLH = true;
}

void GPSData::consume(const uublox_msgs::NavVELNED::ConstPtr &msg) {
    if (this->isComplete()) // if we consume on complete data, we re-initialize the data
        this->markDirty();

    // https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/NavVELNED.msg
    this->ned_north = msg->velN; // NED north velocity [cm/s]
    this->ned_east = msg->velE; // NED east velocity [cm/s]
    this->ned_down = msg->velD; // NED down velocity [cm/s]
    this->speed_2d = msg->gSpeed; // Ground Speed (2-D) [cm/s]
    this->heading_2d = msg->heading; // Heading of motion 2-D [deg / 1e-5]

    this->hasVELNED = true;
}

void GPSData::consume(const uublox_msgs::NavSOL::ConstPtr &msg) {
    if (this->isComplete()) // if we consume on complete data, we re-initialize the data
        this->markDirty();

    // https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/NavSOL.msg
    this->fix_type = msg->gpsFix;
    this->fix_status = msg->flags;
    this->position_DOP = msg->pDOP; // [1 / 0.01]
    this->satellites = msg->numSV;

    this->hasSOLUTION = true;
}

bool GPSData::isComplete() {
    return this->hasPOSLLH && this->hasSOLUTION && this->hasVELNED;
}

void GPSData::markDirty() {
    this->hasPOSLLH = this->hasSOLUTION = this->hasVELNED = false;
}

bool GPSData::has3DLock() {
    return this->fix_type == 0x03;
}

void GPSData::setTime(uint32_t time) {
    this->time = time;
}

ubx_nav_posllh GPSData::getPOSLLH() {
    ubx_nav_posllh msg_posllh;
    msg_posllh.time = this->gps_time;
    msg_posllh.longitude = this->longitude;
    msg_posllh.latitude = this->latitude;
    msg_posllh.altitude_msl = this->altitude_msl;
    return msg_posllh;
}

ubx_nav_status GPSData::getSTATUS() {
    ubx_nav_status msg_status;
    msg_status.time = this->gps_time;
    msg_status.fix_type = this->fix_type;
    msg_status.fix_status = this->fix_status;
    return msg_status;
}

ubx_nav_solution GPSData::getSOLUTION() {
    ubx_nav_solution msg_solution;
    msg_solution.time = this->gps_time;
    msg_solution.week = this->week;
    msg_solution.fix_type = this->fix_type;
    msg_solution.fix_status = this->fix_status;
    msg_solution.position_DOP = this->position_DOP;
    msg_solution.satellites = this->satellites;
    return msg_solution;
}

ubx_nav_velned GPSData::getVELNED() {
    ubx_nav_velned msg_velned;
    msg_velned.time = this->gps_time;
    msg_velned.ned_north = this->ned_north;
    msg_velned.ned_east = this->ned_east;
    msg_velned.ned_down = this->ned_down;
    msg_velned.speed_2d = this->speed_2d;
    msg_velned.heading_2d = this->heading_2d;
    return msg_velned;
}

void GPSData::printData() {

}