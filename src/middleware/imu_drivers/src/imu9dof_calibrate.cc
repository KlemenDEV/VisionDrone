#include <ros/ros.h>

#include "MPU9250.h"

MPU9250 imu;

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu9dof_calibrate");
    ros::NodeHandle nh;

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

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    do {
        cout << '\n' << "Press a key to start acc/gyr calibration, hold the sensor still...";
    } while (cin.get() != '\n');

    imu.calibrateAccelGyro();

    do {
        cout << '\n' << "Press a key to start mag calibration, rotate the sensor around all axes for ~20 seconds...";
    } while (cin.get() != '\n');

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    imu.calibrateMag();

    i2c_stop(&dev);
    i2c_stop(&dev_mag);

    printf("=====================================\n");
    printf("< calibration parameters >\n");
    printf("accel bias [g]: ");
    printf("%.12f", imu.getAccBiasX() * 1000.f / (float) MPU9250::CALIB_ACCEL_SENSITIVITY);
    printf(", ");
    printf("%.12f", imu.getAccBiasY() * 1000.f / (float) MPU9250::CALIB_ACCEL_SENSITIVITY);
    printf(", ");
    printf("%.12f", imu.getAccBiasZ() * 1000.f / (float) MPU9250::CALIB_ACCEL_SENSITIVITY);
    printf("\n");
    printf("gyro bias [deg/s]: ");
    printf("%.12f", imu.getGyroBiasX() / (float) MPU9250::CALIB_GYRO_SENSITIVITY);
    printf(", ");
    printf("%.12f", imu.getGyroBiasY() / (float) MPU9250::CALIB_GYRO_SENSITIVITY);
    printf(", ");
    printf("%.12f", imu.getGyroBiasZ() / (float) MPU9250::CALIB_GYRO_SENSITIVITY);
    printf("\n");
    printf("mag bias [mG]: ");
    printf("%.12f", imu.getMagBiasX());
    printf(", ");
    printf("%.12f", imu.getMagBiasY());
    printf(", ");
    printf("%.12f", imu.getMagBiasZ());
    printf("\n");
    printf("mag scale []: ");
    printf("%.12f", imu.getMagScaleX());
    printf(", ");
    printf("%.12f", imu.getMagScaleY());
    printf(", ");
    printf("%.12f", imu.getMagScaleZ());
    printf("\n");

    printf("=====================================\n");
    printf("< RAW parameters >\n");
    printf("accel bias: ");
    printf("%.12f", imu.getAccBiasX());
    printf(", ");
    printf("%.12f", imu.getAccBiasY());
    printf(", ");
    printf("%.12f", imu.getAccBiasZ());
    printf("\n");
    printf("gyro bias: ");
    printf("%.12f", imu.getGyroBiasX());
    printf(", ");
    printf("%.12f", imu.getGyroBiasY());
    printf(", ");
    printf("%.12f", imu.getGyroBiasZ());
    printf("\n");
    printf("mag bias: ");
    printf("%.12f", imu.getMagBiasX());
    printf(", ");
    printf("%.12f", imu.getMagBiasY());
    printf(", ");
    printf("%.12f", imu.getMagBiasZ());
    printf("\n");
    printf("mag scale: ");
    printf("%.12f", imu.getMagScaleX());
    printf(", ");
    printf("%.12f", imu.getMagScaleY());
    printf(", ");
    printf("%.12f", imu.getMagScaleZ());
    printf("\n");

    ros::shutdown();

    return 0;
}