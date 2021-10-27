#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

int main(int argc, char **argv) {
    ros::init(argc, argv, "sonar_node");
    ros::NodeHandle nh;

    ros::Publisher sonar_pub = nh.advertise<sensor_msgs::Range>("/sonar/data", 10);

    int fd = open("/dev/ttyAMA3", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        std::cerr << "Error opening sonar tty\n";
        return -1;
    }

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

    // set baud rate to 9600
    cfsetispeed(&tty, B9600);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    char rxBuf[80];
    char data;

    int i = 0;
    bool validData = false;

    while (ros::ok()) {
        while (read(fd, &data, 1)) {
            if (data == 'R') {
                i = 0;
            } else {
                rxBuf[i - 1] = data;
                if (rxBuf[i - 1] == 10) {
                    rxBuf[i - 1] = 0;
                    validData = true;
                    break;
                }
                i++;
            }
        }

        if (validData) {
            int range_inch = atoi(&rxBuf[0]);

            sensor_msgs::Range rangeMsg;

            if (range_inch <= 6 || range_inch >= 255) {
                rangeMsg.range = -1;
            } else {
                rangeMsg.range = range_inch * 0.0254; // inch to m
            }

            rangeMsg.header.stamp = ros::Time::now();
            sonar_pub.publish(rangeMsg);

            validData = false;
        }
    }

    close(fd);

    ros::shutdown();

    return 0;
}