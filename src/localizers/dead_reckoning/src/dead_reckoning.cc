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
#include <cmath>

ros::Publisher publisher_velocity;

double t_last = -1;

double vx = 0, vy = 0;

double height_last = 0;
int hcount = 0;

void heightCallback(const std_msgs::Float64::ConstPtr &msg) {
    height_last = msg->data;
}

void imuDataCallback(const sensor_msgs::Imu::ConstPtr &imu_msg) {
    tf2::Quaternion quat_est_rot;
    tf2::fromMsg(imu_msg->orientation, quat_est_rot);
    tf2::Matrix3x3 est_rot(quat_est_rot);
    double roll, pitch, yaw;
    est_rot.getRPY(roll, pitch, yaw);

    if (t_last == -1) {
        if (height_last > 6)
            hcount++;

        if (hcount > 50)
            t_last = imu_msg->header.stamp.toSec();
        return;
    }

    double dt = imu_msg->header.stamp.toSec() - t_last;

    double ax = imu_msg->linear_acceleration.x - 0.034772;
    double ay = imu_msg->linear_acceleration.y - 0.163474;
    double az = imu_msg->linear_acceleration.z;

    double axl = ax * cos(pitch) + ay * sin(roll) * sin(pitch) - az * cos(roll) * sin(pitch);
    double ayl = ax * 0 + ay * cos(roll) + az * sin(roll);

    if (std::abs(ax) > 0.4)
        vy += axl * dt;

    if (std::abs(ay) > 0.4)
        vx += ayl * dt;

    geometry_msgs::TwistWithCovarianceStamped velocitymsg;
    velocitymsg.header.frame_id = "uav_velocity";
    velocitymsg.header.stamp = imu_msg->header.stamp;
    velocitymsg.twist.twist.linear.x = -vy;
    velocitymsg.twist.twist.linear.y = -vx;
    publisher_velocity.publish(velocitymsg);

    t_last = imu_msg->header.stamp.toSec();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "dead_reckoning");
    ros::NodeHandle nh;

    ros::Subscriber sub_imu2 = nh.subscribe("/imu/9dof", 15, imuDataCallback);
    publisher_velocity = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/dead_reckoning/velocity_out", 1);

    ros::Subscriber sub_height = nh.subscribe("/drone/height_estimate", 1, heightCallback);

    ros::spin();
    ros::shutdown();

    return 0;
}