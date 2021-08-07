#include <ros/ros.h>

#include "MPU9250.h"

MPU9250 imu;

int main(int argc, char **argv) {
    ros::init(argc, argv, "mag_driver_node");
    ros::NodeHandle nh;

    struct I2cDevice dev;
    dev.filename = "/dev/i2c-1";
    dev.addr = MPU9250_DEFAULT_ADDRESS;
    if (i2c_start(&dev)) {
        printf("failed to start i2c device 1\r\n");
        return -11;
    }

    struct I2cDevice dev_mag;
    dev_mag.filename = "/dev/i2c-1";
    dev_mag.addr = AK8963_ADDRESS;
    if (i2c_start(&dev_mag)) {
        printf("failed to start i2c device 2\r\n");
        return -11;
    }

    imu.setup(&dev, &dev_mag);

    ros::Rate loop_rate(200);
    while (ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
    }

    i2c_stop(&dev);

    ros::shutdown();

    return 0;
}