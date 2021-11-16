#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <PoseManager.h>

double px = 0, py = 0;

ros::Time time_last;
bool initialzed = false;

PoseManager *poseManager;

ros::Publisher pub_vel_enu;

#define MAX_VEL 14 // m/s

void velocityCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg) {
    if (isnan(msg->twist.covariance[0])) {
        initialzed = false;
        ROS_WARN_THROTTLE(1, "Velocity integration disabled, invalid data received");
        return;
    }

    if (!initialzed) {
        time_last = msg->header.stamp;
        initialzed = true;
        return;
    }

    auto dt = (double) (msg->header.stamp - time_last).toSec();

    double vel_x = msg->twist.twist.linear.x;
    double vel_y = msg->twist.twist.linear.y;

    if (vel_x > MAX_VEL) vel_x = MAX_VEL;
    else if (vel_x < -MAX_VEL) vel_x = -MAX_VEL;

    if (vel_y > MAX_VEL) vel_y = MAX_VEL;
    else if (vel_y < -MAX_VEL) vel_y = -MAX_VEL;

    double vel_enu_x = vel_x * cos(-poseManager->yaw_last) - vel_y * sin(-poseManager->yaw_last);
    double vel_enu_y = vel_x * sin(-poseManager->yaw_last) + vel_y * cos(-poseManager->yaw_last);

    px += vel_enu_x * dt;
    py += vel_enu_y * dt;

    time_last = msg->header.stamp;

    poseManager->publishData(px, py);

    geometry_msgs::TwistWithCovarianceStamped velocitymsg;
    velocitymsg.header.frame_id = "uav_velocity_enu";
    velocitymsg.header.stamp = msg->header.stamp;
    velocitymsg.twist.twist.linear.x = vel_enu_x;
    velocitymsg.twist.twist.linear.y = vel_enu_y;
    pub_vel_enu.publish(velocitymsg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "velocity_integrator_node");
    ros::NodeHandle nh;

    PoseManager poseManagerObj(&nh);
    poseManager = &poseManagerObj;

    ros::Subscriber velocity_sub = nh.subscribe("/estimate/velocity", 5, velocityCallback);

    pub_vel_enu = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/estimate/velocity_enu", 5);

    ros::spin();
    ros::shutdown();

    return 0;
}
