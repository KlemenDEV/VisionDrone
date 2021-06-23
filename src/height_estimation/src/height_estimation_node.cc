#include <ros/ros.h>

#include <chrono>

#include <sensor_msgs/FluidPressure.h>
#include <std_msgs/Float64.h>

#include <sensor_msgs/Imu.h>

#include "altitude.h"

#define P0 101200
#define G 9.80665

using namespace std;

AltitudeEstimator *altitude;

ros::Publisher height_pub;

std::chrono::time_point<std::chrono::high_resolution_clock> time_last;
bool init = false;

float h_baro = 0;

float accel[3] = {};
float gyro[3] = {};
bool imu_data = false;

void updateData() {
    if (h_baro == 0 || !imu_data)
        return;

    if (!init) {
        time_last = std::chrono::high_resolution_clock::now();
        init = true;
        return;
    }

    std::chrono::time_point<std::chrono::high_resolution_clock> new_time = std::chrono::high_resolution_clock::now();

    std::chrono::duration<float> dt = new_time - time_last;
    altitude->estimate(accel, gyro, h_baro, dt.count());

    time_last = new_time;
}

void baroCallback(const sensor_msgs::FluidPressure::ConstPtr &msg) {
    h_baro = (float) (44330 * (1 - pow(msg->fluid_pressure / P0, 1 / 5.255)));
    updateData();
}

void imuCallback(const sensor_msgs::ImuConstPtr &imu_msg) {
    accel[0] = (float) (imu_msg->linear_acceleration.x / G);
    accel[1] = (float) (imu_msg->linear_acceleration.y / G);
    accel[2] = (float) (imu_msg->linear_acceleration.z / G);

    gyro[0] = (float) imu_msg->angular_velocity.x;
    gyro[1] = (float) imu_msg->angular_velocity.y;
    gyro[2] = (float) imu_msg->angular_velocity.z;

    imu_data = true;
    updateData();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "height_estimation_node");
    ros::NodeHandle nh;

    ros::Subscriber baro_sub = nh.subscribe<sensor_msgs::FluidPressure>
            ("/mavros/imu/static_pressure", 5, baroCallback);

    ros::Subscriber sub_imu = nh.subscribe("/camera/imu", 15, imuCallback);

    height_pub = nh.advertise<std_msgs::Float64>("/drone/height_estimate", 5);

    AltitudeEstimator altitude_local = AltitudeEstimator(0.0173244,    // sigma Accel
                                                         0.00202674666,    // sigma Gyro
                                                         0.01,   // sigma Baro
                                                         0.2,    // ca
                                                         0.5);    // accelThreshold
    altitude = &altitude_local;

    ros::Rate loop_rate(15);
    while (ros::ok()) {
        std_msgs::Float64 fmsg;
        fmsg.data = (float) altitude->getAltitude();
        height_pub.publish(fmsg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::shutdown();

    return 0;
}
