#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>

#include"System.h"

using namespace std;

class ImageGrabber {
public:
    ImageGrabber(ORB_SLAM3::System *pSLAM) : mpSLAM(pSLAM) {}

    void GrabMonoImu(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::ImuConstPtr &imu_msg);

    vector <ORB_SLAM3::IMU::Point> vImuMeas;

    ORB_SLAM3::System *mpSLAM;
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "Mono_inertial3");
    ros::start();
    if (argc != 3) {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono_inertial3 path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, true);
    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;

    message_filters::Subscriber <sensor_msgs::Image> img_sub(nodeHandler, "/camera/image_raw", 1);
    message_filters::Subscriber <sensor_msgs::Imu> imu_sub(nodeHandler, "/imu", 1);

    typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::Image, sensor_msgs::Imu> sync_pol;
    message_filters::Synchronizer <sync_pol> sync(sync_pol(5), img_sub, imu_sub);
    // sync.registerCallback(boost::bind(&ImageGrabber::GrabMonoImu,&igb,_1,_2));
    sync.registerCallback(boost::bind(&ImageGrabber::GrabMonoImu, &igb, _1, _2));


    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}


void ImageGrabber::GrabMonoImu(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::ImuConstPtr &imu_msg) {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    float t = imu_msg->header.stamp.toSec();
    float dx = imu_msg->linear_acceleration.x;
    float dy = imu_msg->linear_acceleration.y;
    float dz = imu_msg->linear_acceleration.z;
    float rx = imu_msg->angular_velocity.x;
    float ry = imu_msg->angular_velocity.y;
    float rz = imu_msg->angular_velocity.z;
    vImuMeas.push_back(ORB_SLAM3::IMU::Point(dx, dy, dz, rx, ry, rz, t));

    mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec(), vImuMeas);
    vImuMeas.clear();
}