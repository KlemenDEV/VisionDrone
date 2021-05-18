#include <Fusion.h>

Fusion::Fusion(ros::NodeHandle *nh) {
    this->point_pub = nh->advertise<nav_msgs::Odometry>("/orbslam3/pose_direct", 5);
}

void Fusion::dataSLAM(const cv::Mat& Tcw) {
    if (Tcw.rows == 4 && Tcw.cols == 4) { // valid data
        cv::Mat Twc(4, 4, CV_32F);
        cv::invert(Tcw, Twc);

        nav_msgs::Odometry odom;
        geometry_msgs::PoseWithCovariance posevc;
        geometry_msgs::Pose pose;
        geometry_msgs::Point pt;
        pt.x = Twc.at<float>(0, 3);
        pt.y = Twc.at<float>(1, 3);
        pt.z = Twc.at<float>(2, 3);
        pose.position = pt;
        posevc.pose = pose;
        odom.pose = posevc;
        odom.header.seq = 1;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "map";
        odom.child_frame_id = "map";
        this->point_pub.publish(odom);
    }
}