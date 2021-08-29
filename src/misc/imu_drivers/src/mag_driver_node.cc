#include <ros/ros.h>

#include <sensor_msgs/MagneticField.h>

#include "i2c.h"

#define HMC5983_ADDRESS 0x1E

int main(int argc, char **argv) {
    ros::init(argc, argv, "mag_driver_node");
    ros::NodeHandle nh;

    ros::Publisher mag_pub = nh.advertise<sensor_msgs::MagneticField>("/mag/data", 15);

    struct I2cDevice dev;
    dev.filename = (char *) "/dev/i2c-1";
    dev.addr = HMC5983_ADDRESS;

    if (i2c_start(&dev)) {
        printf("failed to start i2c device\r\n");
        return -11;
    }

    uint8_t rega[] = {0x00, 0x1C};
    i2c_write(&dev, rega, 2);

    //uint8_t regb[] = {0x01, 0x60};
    //i2c_write(&dev, regb, 2);
    double resolution = 0.92; // mG per lsb

    uint8_t regmode[] = {0x02, 0x00};
    i2c_write(&dev, regmode, 2);

    float mx = 10e8, Mx = -10e8, my = 10e8, My = -10e8, mz = 10e8, Mz = -10e8;
    float ox, oy, oz;

    float offx = 125;
    float offy = -45;
    float offz = 115;

    ros::Rate loop_rate(200);
    while (ros::ok()) {
        uint8_t resetpt[] = {0x03};
        i2c_write(&dev, resetpt, 1);

        uint8_t inbuf[6];
        i2c_read(&dev, inbuf, 6);

        float HX = (inbuf[0] << 8) | inbuf[1];
        float HY = (inbuf[4] << 8) | inbuf[5];
        float HZ = (inbuf[2] << 8) | inbuf[3];
        if (HX > 0x07FF) HX = -(0xFFFF - HX);
        if (HY > 0x07FF) HY = -(0xFFFF - HY);
        if (HZ > 0x07FF) HZ = -(0xFFFF - HZ);

        // z - positive field when up
        // rpy axes

        ///*
        HX -= offx;
        HY -= offy;
        HZ -= offz;
        HX *= resolution * 1e-7; // resolution + mG to T
        HY *= resolution * 1e-7;
        HZ *= resolution * 1e-7;
        sensor_msgs::MagneticField msg;
        msg.header.stamp = ros::Time::now();
        msg.magnetic_field.x = HX;
        msg.magnetic_field.y = HY;
        msg.magnetic_field.z = HZ;
        mag_pub.publish(msg);
        //ROS_WARN("mag: %f, %f, %f", HX, HY, HZ);
        // */

        /*
         * if (mx > HX) mx = HX;
        if (Mx < HX) Mx = HX;
        if (my > HY) my = HY;
        if (My < HY) My = HY;
        if (mz > HZ) mz = HZ;
        if (Mz < HZ) Mz = HZ;
        ox = (Mx + mx) / 2.0;
        oy = (My + my) / 2.0;
        oz = (Mz + mz) / 2.0;
        ROS_WARN("offsets xyz: %f, %f, %f", ox, oy, oz);//*/

        ros::spinOnce();
        loop_rate.sleep();
    }

    i2c_stop(&dev);

    ros::shutdown();

    return 0;
}