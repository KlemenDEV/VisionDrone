#include <iostream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>

#include <opencv2/core/core.hpp>

#include "System.h"
#include "ImuTypes.h"

#include "Fusion.h"

using namespace std;

class ImuGrabber {
public:
    ImuGrabber() = default;

    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber {
public:
    ImageGrabber(ORB_SLAM3::System *pSLAM, ImuGrabber *pImuGb) : mpSLAM(pSLAM), mpImuGb(pImuGb) {}

    void GrabImage(const sensor_msgs::ImageConstPtr &msg);

    static cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);

    void SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> img0Buf;
    std::mutex mBufMutex;

    ORB_SLAM3::System *mpSLAM;
    ImuGrabber *mpImuGb;
};

Fusion *fusion;

bool visualize;

int main(int argc, char **argv) {
    ros::init(argc, argv, "ORBSLAM3");
    ros::NodeHandle nh("~");

    fusion = new Fusion(&nh);

    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    nh.param<bool>("visualize", visualize, true);

    std::string path_to_vocabulary;
    nh.param<std::string>("path_to_vocabulary", path_to_vocabulary,
                          "/home/pylo/drone_ws/non_ros/orb_slam3/Vocabulary/ORBvoc.txt");
    std::string path_to_settings;
    nh.param<std::string>("path_to_settings", path_to_settings,
                          "/home/pylo/drone_ws/src/orb_slam3/orb_slam3.yaml");
    ORB_SLAM3::System SLAM(path_to_vocabulary, path_to_settings, ORB_SLAM3::System::IMU_MONOCULAR, visualize);

    ImuGrabber imugb;
    ImageGrabber igb(&SLAM, &imugb);

    std::string topic_imu;
    nh.param<std::string>("topic_imu", topic_imu, "/camera/imu");
    ros::Subscriber sub_imu = nh.subscribe(topic_imu, 15, &ImuGrabber::GrabImu, &imugb);

    std::string topic_img;
    nh.param<std::string>("topic_img", topic_img, "/camera/infra1/image_rect_raw");
    ros::Subscriber sub_img = nh.subscribe(topic_img, 1, &ImageGrabber::GrabImage, &igb);

    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    ros::spin();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &img_msg) {
    mBufMutex.lock();
    if (!img0Buf.empty())
        img0Buf.pop();
    img0Buf.push(img_msg);
    mBufMutex.unlock();
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg) {
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    return cv_ptr->image.clone();
}

void ImageGrabber::SyncWithImu() {
    cv::Mat im;
    double tIm;

    while (ros::ok()) {
        if (!img0Buf.empty() && !mpImuGb->imuBuf.empty()) {
            tIm = img0Buf.front()->header.stamp.toSec();
            if (tIm > mpImuGb->imuBuf.back()->header.stamp.toSec())
                continue;

            {
                this->mBufMutex.lock();
                im = GetImage(img0Buf.front());
                img0Buf.pop();
                this->mBufMutex.unlock();
            }

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            mpImuGb->mBufMutex.lock();
            if (!mpImuGb->imuBuf.empty()) {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!mpImuGb->imuBuf.empty() &&
                       mpImuGb->imuBuf.front()->header.stamp.toSec() <= tIm) {
                    double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
                    cv::Point3f acc((float) mpImuGb->imuBuf.front()->linear_acceleration.x,
                                    (float) mpImuGb->imuBuf.front()->linear_acceleration.y,
                                    (float) mpImuGb->imuBuf.front()->linear_acceleration.z);
                    cv::Point3f gyr((float) mpImuGb->imuBuf.front()->angular_velocity.x,
                                    (float) mpImuGb->imuBuf.front()->angular_velocity.y,
                                    (float) mpImuGb->imuBuf.front()->angular_velocity.z);
                    vImuMeas.emplace_back(acc, gyr, t);
                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();

            cv::Mat Tcw = mpSLAM->TrackMonocular(im, tIm, vImuMeas);

            fusion->dataSLAM(mpSLAM, Tcw);

            // TODO: if visualisation, publish point cloud from atlas -> maps -> map points
        }

        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}
