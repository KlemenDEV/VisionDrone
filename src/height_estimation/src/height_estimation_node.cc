#include <ros/ros.h>

#include <chrono>

#include <sensor_msgs/FluidPressure.h>
#include <std_msgs/Float64.h>

#include <sensor_msgs/Imu.h>

#include "altitude.h"

#define P0 101200
#define G 9.81

using namespace std;

AltitudeEstimator *altitude;

ros::Publisher height_pub;

std::chrono::time_point<std::chrono::high_resolution_clock> time_last;
bool init = false;

float h_baro = 0;

float accel[3] = {};
float gyro[3] = {};
bool imu_data = false;

bool relative;
float altInit;
int initCounter = 0;
const int counterMax = 10;

void updateData() {
    if (h_baro == 0 || !imu_data || initCounter <= counterMax)
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
    float abs_h_baro = (float) (44330 * (1 - pow(msg->fluid_pressure / P0, 1 / 5.255)));

    if (relative) {
        if (initCounter < counterMax) {
            initCounter++;
            altInit += abs_h_baro;
        } else if (initCounter == counterMax) {
            altInit /= (float) initCounter;
            initCounter++;
        } else {
            h_baro = abs_h_baro - altInit;
            updateData();
        }
    } else {
        h_baro = abs_h_baro;
        updateData();
    }

}

#define ACC_B_X 0.03285804018378258
#define ACC_B_Y -0.12473473697900772
#define ACC_B_Z 0.2566709816455841

#define GYR_B_X -0.00011866784916492179
#define GYR_B_Y 6.184831363498233e-06
#define GYR_B_Z 2.998005766130518e-05

void imuCallback(const sensor_msgs::ImuConstPtr &imu_msg) {
    // convert to earth-centered inertial coordinate system

    accel[0] = (float) ((imu_msg->linear_acceleration.z - ACC_B_Z) / G); // x
    accel[1] = (float) (-(imu_msg->linear_acceleration.x - ACC_B_X) / G); // y
    accel[2] = (float) (-(imu_msg->linear_acceleration.y - ACC_B_Y) / G); // z

    gyro[0] = (float) (imu_msg->angular_velocity.z - GYR_B_Z); // x
    gyro[1] = (float) -(imu_msg->angular_velocity.x - GYR_B_X); // y
    gyro[2] = (float) -(imu_msg->angular_velocity.y - GYR_B_Y); // z

    imu_data = true;
    updateData();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "height_estimation_node");
    ros::NodeHandle nh;
    ros::NodeHandle n("~");

    ros::Subscriber baro_sub = nh.subscribe<sensor_msgs::FluidPressure>
            ("/mavros/imu/static_pressure", 5, baroCallback);

    ros::Subscriber sub_imu = nh.subscribe("/camera/imu", 20, imuCallback);

    height_pub = nh.advertise<std_msgs::Float64>("/drone/height_estimate", 5);

    AltitudeEstimator altitude_local = AltitudeEstimator(0.0173244,    // sigma Accel
                                                         0.00202674666,    // sigma Gyro
                                                         0.01,   // sigma Baro
                                                         0.2,    // ca
                                                         0.5);    // accelThreshold
    altitude = &altitude_local;

    n.param<bool>("relative", relative, false);


    ros::Rate loop_rate(15);
    while (ros::ok()) {
        float alt = (float) altitude->getAltitude();
        if (alt != 0) {
            std_msgs::Float64 fmsg;
            fmsg.data = alt;
            height_pub.publish(fmsg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::shutdown();

    return 0;
}
