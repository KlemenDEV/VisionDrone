#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <PoseManager.h>

float px, py;

ros::Time time_last;
bool initialzed = false;

PoseManager *poseManager;

void velocityCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg) {
    if (!poseManager->datum_set)
        return;

    if (!initialzed) {
        time_last = msg->header.stamp;
        initialzed = true;
        return;
    }

    auto dt = (float) (msg->header.stamp - time_last).toSec();

    px += msg->twist.twist.linear.x * dt;
    py += msg->twist.twist.linear.y * dt;

    time_last = msg->header.stamp;

    poseManager->publishData(px, -py);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "velocity_integrator_node");
    ros::NodeHandle nh;

    PoseManager poseManagerObj(&nh);
    poseManager = &poseManagerObj;

    ros::Subscriber velocity_sub = nh.subscribe("/optic_flow/velocity_out", 5, velocityCallback);

    ros::spin();
    ros::shutdown();

    return 0;
}
