#include "Fusion.h"

#define ENABLE_SLAM
#define ENABLE_DEAD_RECKONING

Fusion::Fusion(ros::NodeHandle *nh) {
    poseManager = new PoseManager(nh);

    imu1.subscribe(*nh, "/imu/9dof", 15);
    imu2.subscribe(*nh, "/camera/imu", 15);
    syncptr.reset(new syncer(syncPolicy(15), imu1, imu2));
    syncptr->registerCallback(boost::bind(&Fusion::imuDataCallback, this, _1, _2));

    // pre-init states
    velocity = Eigen::Vector3d::Zero();
    aBias = Eigen::Vector3d(0.03285804018378258, -0.12473473697900772, 0.2566709816455841);

    new std::thread(&Fusion::tracking, this);
}

void Fusion::tracking() {
    time_last = std::chrono::high_resolution_clock::now();

    while (ros::ok()) {
#ifdef ENABLE_DEAD_RECKONING
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(
                std::chrono::high_resolution_clock::now() - time_last).count();

        if (elapsed_time_ms > 400) {
            deadReckoning();
        }
#endif
    }
}

void Fusion::dataSLAM(ORB_SLAM3::System *mpSLAM, const cv::Mat &Tcw, vector <ORB_SLAM3::IMU::Point> &vImuMeas) {
    if (Tcw.rows == 4 && Tcw.cols == 4) { // valid data
        cv::Mat Twc(4, 4, CV_32F);

        cv::invert(Tcw, Twc);

        float slam_x = Twc.at<float>(0, 3);
        float slam_y = Twc.at<float>(1, 3);

        if (state == SLAM_TRACKING) {
            if (mpSLAM->GetTimeFromIMUInit() <= 0 || mpSLAM->mpTracker->mState != ORB_SLAM3::Tracking::OK) {
                ROS_WARN("SLAM data not trusted!");
                state = DEAD_RECKONING;
                return;
            }

            float dx = slam_x - slam_ox;
            float dy = slam_y - slam_oy;

            // filter out big jumps (eg. due to map re-alignments)
            float dxy = sqrt(dx * dx + dy * dy);
            if (dxy > 4) {
                ROS_WARN("Large jump detected, will not use SLAM data!");
                state = DEAD_RECKONING;
                return;
            }

            ROS_WARN_THROTTLE(1, "SLAM");

            auto s = (float) sin(yaw_offset);
            auto c = (float) cos(yaw_offset);
            px += c * dx - s * dy;
            py += s * dx + c * dy;

            slam_ox = slam_x;
            slam_oy = slam_y;

            imuBuffer.clear(); // we got slam data, we do not need imu data for dead reckoning

            time_last = std::chrono::high_resolution_clock::now();

            poseManager->publishData(px, py);
        } else if (mpSLAM->GetTimeFromIMUInit() > 0 && mpSLAM->mpTracker->mState == ORB_SLAM3::Tracking::OK) {
#ifdef ENABLE_SLAM
            if (state == DEAD_RECKONING) {
                tf2::Matrix3x3 tf2_rot(Twc.at<float>(0, 0), Twc.at<float>(0, 1), Twc.at<float>(0, 2),
                                       Twc.at<float>(1, 0), Twc.at<float>(1, 1), Twc.at<float>(1, 2),
                                       Twc.at<float>(2, 0), Twc.at<float>(2, 1), Twc.at<float>(2, 2));

                double roll2, pitch2, yaw2;
                tf2_rot.getRPY(roll2, pitch2, yaw2, 1);

                yaw_offset = poseManager->yaw_mag_curr - yaw2;
            }

            state = SLAM_TRACKING;

            slam_ox = slam_x;
            slam_oy = slam_y;
            ROS_WARN("Back to SLAM, yaw offset: %f", yaw_offset);

            time_last = std::chrono::high_resolution_clock::now();
#endif
        }
    }
}

void Fusion::deadReckoning() {
    static double dt = 1. / 150.;

    if (state == IDLE) return; // require slam tracking first

    if (state != DEAD_RECKONING) {
        velocity = Eigen::Vector3d::Zero();
        state = DEAD_RECKONING;
        ROS_WARN("(Re)Initiating dead reckoning");
    }

    if (imuBuffer.empty()) return;

    ROS_WARN_THROTTLE(1, "Dead reckoning");

    Eigen::Vector3d position = Eigen::Vector3d::Zero();

    double t_last = -1;

    Eigen::Vector3d gravity = Eigen::Vector3d(0, 0, 9.81);

    for (const auto &pt : imuBuffer) {
        if (t_last != -1)
            dt = pt.header.stamp.toSec() - t_last;

        Eigen::Quaterniond orientation;
        tf::quaternionMsgToEigen(pt.orientation, orientation);

        Eigen::Vector3d aBiasCorrected = Eigen::Vector3d(
                (pt.linear_acceleration.z + 0.2566709816455841),
                -(pt.linear_acceleration.x + 0.03285804018378258),
                -(pt.linear_acceleration.y - 0.12473473697900772)
        );

        Eigen::Vector3d aPlanar = (orientation.toRotationMatrix() * aBiasCorrected) - gravity;

        Eigen::Vector3d aPlanarDeadband = Eigen::Vector3d(
                abs(aPlanar.x()) < 0.3 ? 0 : aPlanar.x(),
                abs(aPlanar.y()) < 0.3 ? 0 : aPlanar.y(),
                abs(aPlanar.z()) < 0.3 ? 0 : aPlanar.z()
        );

        position += velocity * dt;
        velocity += aPlanarDeadband * dt;

        t_last = pt.header.stamp.toSec();
    }
    imuBuffer.clear();

    px -= (float) position.y();
    py -= (float) position.x();

    poseManager->publishData(px, py);
}

void Fusion::imuDataCallback(const sensor_msgs::Imu::ConstPtr &imu_msg, const sensor_msgs::Imu::ConstPtr &imu_msg2) {
    imuMutex.lock();

    sensor_msgs::Imu fused;
    fused.orientation = imu_msg->orientation;
    fused.linear_acceleration = imu_msg2->linear_acceleration;
    fused.header = imu_msg->header;
    imuBuffer.push_back(fused);

    imuMutex.unlock();
}
