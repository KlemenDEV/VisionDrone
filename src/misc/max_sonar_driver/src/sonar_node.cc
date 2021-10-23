#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#include <fcntl.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "mag_driver_node");
    ros::NodeHandle nh;

    ros::Publisher sonar_pub = nh.advertise<sensor_msgs::Range>("/sonar/data", 10);
    
    int fd = open("/dev/ttyUSB0", O_EXCL);
    if (fd < 0) {
        std::cerr << "Error opening sonar tty\n";
        return -1;
    }

    char rxBuf[80];
    while (ros::ok()) {
        int i = 0;
        while (read(fd, &rxBuf[i], 1)) {
            if (rxBuf[i] == '\r') {
                rxBuf[i] = '\0';
                break;
            }
            i++;
        }

        sensor_msgs::Range rangeMsg;
        rangeMsg.range = strtof(&rxBuf[1], NULL) / 1000.0;
        rangeMsg.header.stamp = ros::Time::now();
        sonar_pub.publish(rangeMsg);
    }

    ros::shutdown();

    return 0;
}