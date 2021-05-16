#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"System.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <nav_msgs/Odometry.h>

using namespace std;

class ImageGrabber {
public:
    ImageGrabber(ORB_SLAM3::System *pSLAM) : mpSLAM(pSLAM) {}

    void GrabImage(const sensor_msgs::ImageConstPtr &msg);

    ORB_SLAM3::System *mpSLAM;
};

ros::Publisher point_pub;

// Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings
int main(int argc, char **argv) {
    ros::init(argc, argv, "Mono");
    ros::start();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);

    point_pub = nh.advertise<nav_msgs::Odometry>("/slam/pose", 1000);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());

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
        point_pub.publish(odom);

        std::cout << std::to_string(Twc.at<float>(0, 3)) << ", "
                  << std::to_string(Twc.at<float>(1, 3)) << ", "
                  << std::to_string(Twc.at<float>(2, 3)) << std::endl;
    }
}


