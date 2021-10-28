#include <ros/ros.h>

#include <sensor_msgs/FluidPressure.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

#include <chrono>

#include "altitude.h"

#define P0 101200.0

#define G 9.81

#define ACC_B_X -0.034772
#define ACC_B_Y -0.163474
#define ACC_B_Z  0.058925

#define GYR_B_X -0.003426
#define GYR_B_Y  0.003331
#define GYR_B_Z -0.002723

AltitudeEstimator *altitude;

ros::Publisher height_pub;
ros::Publisher height_pub_abs;

std::chrono::time_point <std::chrono::high_resolution_clock> time_last;
bool init = false;

volatile float h_baro = 0;
volatile float accel[3] = {0};
volatile float gyro[3] = {0};

bool imu_data = false;

void updateData() {
    if (h_baro == 0 || !imu_data)
        return;

    std::chrono::time_point <std::chrono::high_resolution_clock> new_time = std::chrono::high_resolution_clock::now();

    if (!init) {
        time_last = new_time;
        init = true;
        return;
    }

    std::chrono::duration<float> dt = new_time - time_last;


    altitude->estimate(const_cast<float*>(accel), const_cast<float*>(gyro), h_baro, dt.count());

    time_last = new_time;
}

void baroCallback(const sensor_msgs::FluidPressure::ConstPtr &msg) {
    h_baro = (float) (44330.0 * (1 - pow(msg->fluid_pressure / P0, 1 / 5.255)));
}

void imuCallback(const sensor_msgs::ImuConstPtr &imu_msg) {
    accel[0] = (float) (imu_msg->linear_acceleration.x - ACC_B_X) / G; // x
    accel[1] = (float) (imu_msg->linear_acceleration.y - ACC_B_Y) / G; // y
    accel[2] = (float) (imu_msg->linear_acceleration.z - ACC_B_Z) / G; // z

    gyro[0] = (float) (imu_msg->angular_velocity.x - GYR_B_X); // x
    gyro[1] = (float) (imu_msg->angular_velocity.y - GYR_B_Y); // y
    gyro[2] = (float) (imu_msg->angular_velocity.z - GYR_B_Z); // z

    imu_data = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "height_estimation_node");
    ros::NodeHandle nh;

    ros::Subscriber baro_sub = nh.subscribe<sensor_msgs::FluidPressure>("/baro/data", 5, baroCallback);
    ros::Subscriber sub_imu = nh.subscribe("/imu/9dof", 10, imuCallback);

    height_pub = nh.advertise<std_msgs::Float64>("/drone/height_estimate", 5);

    altitude = new AltitudeEstimator(1.5518791653745640e-02,    // sigma Accel
                                     1.2863346079614393e-03,    // sigma Gyro
                                     0.0005,   // sigma Baro
                                     0.5,    // ca
                                     0.6);    // accelThreshold

    ros::Rate loop_rate(30);

    double rel_init = 0;
    int init_counter = 0;

    while (ros::ok()) {
        updateData();

        float alt = (float) altitude->getAltitude();
        if (alt != 0 || init_counter > 200) {
            if (init_counter < 200) {
                init_counter++;
            } else if (init_counter == 200) {
                rel_init = alt;
                init_counter++;
                ROS_WARN("Relative height origin: %f", alt);
            } else {
                std_msgs::Float64 fmsg_rel;
                fmsg_rel.data = alt - rel_init;
                height_pub.publish(fmsg_rel);
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::shutdown();

    return 0;
}
