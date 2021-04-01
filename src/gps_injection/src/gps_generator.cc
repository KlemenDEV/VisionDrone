#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>

#include <stdio.h>
#include <string.h>

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <ros/ros.h>

#include "ubx.h"

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "GPSGenerator");
    ros::NodeHandle nh;

    ros::Rate loop_rate(5);

    // Open the serial port
    // https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
    int serial_port = open("/dev/ttyAMA3", O_RDWR | O_NOCTTY | O_NDELAY);

    // Create new termios structure
    struct termios tty;

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    // set baud rate to 38400 (UBX standard)
    cfsetispeed(&tty, B38400);

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return -1;
    }

    uint32_t gps_time = 0;

    while (ros::ok()) {
        ubx_nav_posllh msg_posllh;
        msg_posllh.time = gps_time;
        msg_posllh.longitude = 0;
        msg_posllh.latitude = 0;
        msg_posllh.altitude_msl = 0;
        std::vector<uint8_t> msg_posllh_full = generateUBXMessage(UBX_CLASS_NAV, UBX_NAV_POSLLH_ID,
                                                                  reinterpret_cast<uint8_t *>(&msg_posllh),
                                                                  sizeof(ubx_nav_posllh));
        write(serial_port, msg_posllh_full.data(), msg_posllh_full.size());
        ros::Duration(0.1).sleep();

        /*ubx_nav_status msg_status;
        msg_status.time = gps_time;
        msg_status.fix_type = 0x03;
        msg_status.fix_status = 0b1101;
        std::vector<uint8_t> msg_status_full = generateUBXMessage(UBX_CLASS_NAV, UBX_NAV_STATUS_ID,
                                                                  reinterpret_cast<uint8_t *>(&msg_status),
                                                                  sizeof(ubx_nav_status));
        write(serial_port, msg_status_full.data(), msg_status_full.size());
        ros::Duration(0.1).sleep();*/

        ubx_nav_velned msg_velned;
        msg_velned.time = gps_time;
        msg_velned.ned_north = 0;
        msg_velned.ned_east = 0;
        msg_velned.ned_down = 0;
        msg_velned.speed_2d = 0;
        msg_velned.heading_2d = 0;
        std::vector<uint8_t> msg_velned_full = generateUBXMessage(UBX_CLASS_NAV, UBX_NAV_VELNED_ID,
                                                                  reinterpret_cast<uint8_t *>(&msg_velned),
                                                                  sizeof(ubx_nav_velned));
        write(serial_port, msg_velned_full.data(), msg_velned_full.size());
        ros::Duration(0.1).sleep();

        ubx_nav_solution msg_solution;
        msg_solution.time = gps_time;
        msg_solution.week = 1721;
        msg_solution.fix_type = 0x03;
        msg_solution.fix_status = 0b1101;
        msg_solution.position_DOP = 5;
        msg_solution.satellites = 5;
        std::vector<uint8_t> msg_solution_full = generateUBXMessage(UBX_CLASS_NAV, UBX_NAV_SOLUTION_ID,
                                                                    reinterpret_cast<uint8_t *>(&msg_solution),
                                                                    sizeof(ubx_nav_solution));
        write(serial_port, msg_solution_full.data(), msg_solution_full.size());

        // increment GPS time for 200ms (5Hz loop)
        gps_time += 200;

        ros::spinOnce();
        loop_rate.sleep();
    }

    // close the serial port
    close(serial_port);

    ros::shutdown();

    return 0;
}
