#include "PoseManager.h"

static PoseManager *poseManager;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    poseManager->publishData(msg->pose.position.x, msg->pose.position.y);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_manager");
    ros::NodeHandle nh;

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/orbslam3/pose_raw", 1, poseCallback);

    PoseManager poseManagerObj(&nh);
    poseManager = &poseManagerObj;

    ros::spin();
    ros::shutdown();

    return 0;
}