#include "GPSData.h"

void GPSData::consume(const ublox_msgs::NavPOSLLH::ConstPtr &msg) {
    if (this->isComplete()) // if we consume on complete data, we re-initialize the data
        this->markDirty();

    // https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/NavPOSLLH.msg
    this->longitude = msg->lon; // [deg / 1e-7]
    this->latitude = msg->lat; // [deg / 1e-7]
    this->altitude_msl = msg->hMSL; // Height above mean sea level [mm]

    this->hasPOSLLH = true;
}

void GPSData::consume(const ublox_msgs::NavVELNED::ConstPtr &msg) {
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

void GPSData::consume(const ublox_msgs::NavSOL::ConstPtr &msg) {
    if (this->isComplete()) // if we consume on complete data, we re-initialize the data
        this->markDirty();

    // https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/NavSOL.msg
    this->fix_type = msg->gpsFix;
    this->fix_status = msg->flags;
    this->position_DOP = msg->pDOP; // [1 / 0.01]
    this->satellites = msg->numSV;
    this->week = 1721;

    this->hasSOLUTION = true;
}

bool GPSData::isComplete() {
    return this->hasPOSLLH && this->hasSOLUTION && this->hasVELNED;
}

void GPSData::markDirty() {
    this->hasPOSLLH = false;
    this->hasSOLUTION = false;
    this->hasVELNED = false;
}

bool GPSData::has3DLock() {
    return this->fix_type == 0x03;
}

void GPSData::setTime(uint32_t time) {
    this->time = time;
}

ubx_nav_posllh GPSData::getPOSLLH() {
    ubx_nav_posllh msg_posllh;
    msg_posllh.time = this->time;
    msg_posllh.longitude = this->longitude;
    msg_posllh.latitude = this->latitude;
    msg_posllh.altitude_msl = this->altitude_msl;
    return msg_posllh;
}

ubx_nav_status GPSData::getSTATUS() {
    ubx_nav_status msg_status;
    msg_status.time = this->time;
    msg_status.fix_type = this->fix_type;
    msg_status.fix_status = this->fix_status;
    return msg_status;
}

ubx_nav_solution GPSData::getSOLUTION() {
    ubx_nav_solution msg_solution;
    msg_solution.time = this->time;
    msg_solution.week = this->week;
    msg_solution.fix_type = this->fix_type;
    msg_solution.fix_status = this->fix_status;
    msg_solution.position_DOP = this->position_DOP;
    msg_solution.satellites = this->satellites;
    return msg_solution;
}

ubx_nav_velned GPSData::getVELNED() {
    ubx_nav_velned msg_velned;
    msg_velned.time = this->time;
    msg_velned.ned_north = this->ned_north;
    msg_velned.ned_east = this->ned_east;
    msg_velned.ned_down = this->ned_down;
    msg_velned.speed_2d = this->speed_2d;
    msg_velned.heading_2d = this->heading_2d;
    return msg_velned;
}

/**
 * Consumes cartesian data and offsets the base GPSData for this cartesian data and stores
 * in the current GPSData instance.
 *
 * @param base - GPSData to offset from
 * @param x - offset in x axis [m]
 * @param y - offset in y axis (height) [m]
 * @param z - offset in z axis [m]
 * @param vx - velocity along x axis (south to north) [m/s]
 * @param vy - velocity along y axis (up to down) [m/s]
 * @param vz - velocity along z axis (east to west) [m/s]
 * @param heading - heading of the drone relative to the Earth's north
 */
void GPSData::consume(GPSData base, double x, double y, double z, double vx, double vy, double vz, double heading) {
    // TODO: implement this

    // fill out GPS status info
    this->fix_type = 0x03; // 3D lock
    this->fix_status = 0b1101;
    this->position_DOP = 5; // some arbitrary value
    this->satellites = 99; // 99 indicates we are in cartesian prediction mode on quadcopter's video OSD
    this->week = 1721; // some arbitrary value

    // mark complete
    this->hasPOSLLH = true;
    this->hasSOLUTION = true;
    this->hasVELNED = true;
}

void GPSData::printData() {
    // TODO: implement this
}