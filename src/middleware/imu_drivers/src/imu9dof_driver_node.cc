#include <ros/ros.h>

#include <sensor_msgs/Imu.h>

#include "MPU9250.h"

#define G 9.81

MPU9250 imu;

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu9dof_driver_node");
    ros::NodeHandle nh;

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/9dof", 15);

    struct I2cDevice dev;
    dev.filename = "/dev/i2c-1";
    dev.addr = MPU9250_DEFAULT_ADDRESS;
    if (i2c_start(&dev)) {
        printf("failed to start i2c device for imu\r\n");
        return -11;
    }

    struct I2cDevice dev_mag;
    dev_mag.filename = "/dev/i2c-1";
    dev_mag.addr = AK8963_ADDRESS;
    if (i2c_start(&dev_mag)) {
        printf("failed to start i2c device for magnetometer\r\n");
        return -11;
    }

    imu.selectFilter(QuatFilterSel::MADGWICK);
    imu.setFilterIterations(15);
    imu.setMagneticDeclination(4.35);

    imu.setup(&dev, &dev_mag);

    imu.setAccBias(318.243896484375, 218.146347045898, 813.560546875000);
    imu.setGyroBias(-160.829269409180, 106.951217651367, 29.487804412842);
    imu.setMagBias(67.437385559082, 141.586883544922, -13.775641441345);
    imu.setMagScale(1.004357337952, 1.010964989662, 0.985042750835);

    ros::Rate loop_rate(200);
    while (ros::ok()) {
        if (imu.update()) {
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = "imu_9dof";

            // imu returns in deg/s, convert to rad/s
            imu_msg.angular_velocity.x = imu.getGyroX() * DEG_TO_RAD;
            imu_msg.angular_velocity.y = imu.getGyroY() * DEG_TO_RAD;
            imu_msg.angular_velocity.z = imu.getGyroZ() * DEG_TO_RAD;

            // imu returns in g's, convert to m/s^2
            imu_msg.linear_acceleration.x = imu.getLinearAccX() * G;
            imu_msg.linear_acceleration.y = imu.getLinearAccY() * G;
            imu_msg.linear_acceleration.z = imu.getLinearAccZ() * G;

            imu_msg.orientation.x = imu.getQuaternionX();
            imu_msg.orientation.y = imu.getQuaternionY();
            imu_msg.orientation.z = imu.getQuaternionZ();
            imu_msg.orientation.w = imu.getQuaternionW();

            imu_pub.publish(imu_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    i2c_stop(&dev);
    i2c_stop(&dev_mag);

    ros::shutdown();

    return 0;
}