#include "UBXSender.h"

UBXSender::UBXSender() {
    // Open the serial port
    // https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
    this->serial_port = open("/dev/ttyAMA3", O_RDWR | O_NOCTTY | O_NDELAY);

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
}

void UBXSender::sendData(GPSData *data) {
    // Testing showed status message is not strictly needed for APM to operate
    /*ubx_nav_status msg_status = data->getSTATUS();
    std::vector<uint8_t> msg_status_full = generateUBXMessage(UBX_CLASS_NAV, UBX_NAV_STATUS_ID,
                                                              reinterpret_cast<uint8_t *>(&msg_status),
                                                              sizeof(ubx_nav_status));
    write(this->serial_port, msg_status_full.data(), msg_status_full.size());
    this_thread::sleep_for(std::chrono::milliseconds(100));*/

    ubx_nav_posllh msg_posllh = data->getPOSLLH();
    std::vector<uint8_t> msg_posllh_full = generateUBXMessage(UBX_CLASS_NAV, UBX_NAV_POSLLH_ID,
                                                              reinterpret_cast<uint8_t *>(&msg_posllh),
                                                              sizeof(ubx_nav_posllh));
    write(this->serial_port, msg_posllh_full.data(), msg_posllh_full.size());
    this_thread::sleep_for(std::chrono::milliseconds(100));

    ubx_nav_velned msg_velned = data->getVELNED();
    std::vector<uint8_t> msg_velned_full = generateUBXMessage(UBX_CLASS_NAV, UBX_NAV_VELNED_ID,
                                                              reinterpret_cast<uint8_t *>(&msg_velned),
                                                              sizeof(ubx_nav_velned));
    write(this->serial_port, msg_velned_full.data(), msg_velned_full.size());
    this_thread::sleep_for(std::chrono::milliseconds(100));

    ubx_nav_solution msg_solution = data->getSOLUTION();
    std::vector<uint8_t> msg_solution_full = generateUBXMessage(UBX_CLASS_NAV, UBX_NAV_SOLUTION_ID,
                                                                reinterpret_cast<uint8_t *>(&msg_solution),
                                                                sizeof(ubx_nav_solution));
    write(this->serial_port, msg_solution_full.data(), msg_solution_full.size());
}

void UBXSender::close() {
    // close the serial port
    close(this->serial_port);
}