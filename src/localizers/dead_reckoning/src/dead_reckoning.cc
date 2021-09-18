#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <std_msgs/Float64.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geographic_msgs/GeoPose.h>
#include <geographic_msgs/GeoPoint.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>

#include <chrono>
#include <thread>
#include <vector>

ros::Publisher publisher_velocity;


double t_last = -1;
int counter = 0;
volatile double roll, pitch, yaw;

double vx = 0, vy = 0;

void imuDataOrientCallback(const sensor_msgs::Imu::ConstPtr &imu_msg) {
    tf2::Quaternion quat_est_rot;
    tf2::fromMsg(imu_msg->orientation, quat_est_rot);
    tf2::Matrix3x3 est_rot(quat_est_rot);
    double _roll, _pitch, _yaw;
    est_rot.getRPY(_roll, _pitch, _yaw);
    roll = _roll;
    pitch = _pitch;
    yaw = _yaw;
}

void imuDataCallback(const sensor_msgs::Imu::ConstPtr &imu_msg2) {
    if (t_last == -1) {
        t_last = imu_msg2->header.stamp.toSec();
        return;
    }

    double dt = imu_msg2->header.stamp.toSec() - t_last;

    double ax = (imu_msg2->linear_acceleration.z) - 9.81 * sin(roll);
    double ay = (imu_msg2->linear_acceleration.x) - 9.81 * sin(pitch);

    vx -= ax * dt * 0.25;
    vy += ay * dt * 0.25;

    if (counter % 25 == 0) {
        geometry_msgs::TwistWithCovarianceStamped velocitymsg;
        velocitymsg.header.frame_id = "uav_velocity";
        velocitymsg.header.stamp = imu_msg2->header.stamp;
        velocitymsg.twist.twist.linear.x = vx;
        velocitymsg.twist.twist.linear.y = vy;
        publisher_velocity.publish(velocitymsg);
    }

    counter++;
    t_last = imu_msg2->header.stamp.toSec();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "dead_reckoning");

    ros::NodeHandle nh;

    ros::Subscriber sub_imu = nh.subscribe("/imu/9dof", 15, imuDataOrientCallback);
    ros::Subscriber sub_imu2 = nh.subscribe("/camera/imu", 15, imuDataCallback);

    publisher_velocity = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/dead_reckoning/velocity_out", 1);

    ros::spin();
    ros::shutdown();

    return 0;
}