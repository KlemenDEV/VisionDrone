#include "Fusion.h"

Fusion::Fusion(ros::NodeHandle *nh) {
    poseManager = new PoseManager(nh);

    // IMU fused orientation
    imuorient_sub = nh->subscribe<sensor_msgs::Imu>("/camera/imu", 15, &Fusion::imuDataCallback, this);

    // pre-init states
    velocity = Eigen::Vector3d::Zero();
    aBias = Eigen::Vector3d(0.03285804018378258, -0.12473473697900772, 0.2566709816455841);

    new std::thread(&Fusion::tracking, this);
}

void Fusion::tracking() {
    time_last = std::chrono::high_resolution_clock::now();

    while (ros::ok()) {
        if (!poseManager->datum_set)
            continue;

        double elapsed_time_ms = std::chrono::duration<double, std::milli>(
                std::chrono::high_resolution_clock::now() - time_last).count();
        if (elapsed_time_ms > 50) {
            deadReckoning(imuBuffer);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void Fusion::dataSLAM(ORB_SLAM3::System *mpSLAM, const cv::Mat &Tcw, vector<ORB_SLAM3::IMU::Point> &vImuMeas) {
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

            //imuBuffer.clear(); // we got slam data, we do not need imu data for dead reckoning
            //time_last = std::chrono::high_resolution_clock::now();
            //poseManager->publishData(px, py);
        } else if (mpSLAM->GetTimeFromIMUInit() > 0 && mpSLAM->mpTracker->mState == ORB_SLAM3::Tracking::OK) {
            /*if(state == DEAD_RECKONING) {

            }

            state = SLAM_TRACKING;

            slam_ox = slam_x;
            slam_oy = slam_y;
            ROS_WARN("Back to SLAM");*/
        }
    }
}

void Fusion::deadReckoning(const vector<sensor_msgs::ImuConstPtr> &imuBufferCurr) {
    if (imuBufferCurr.empty()) return;

    if (state != DEAD_RECKONING) {
        velocity = Eigen::Vector3d::Zero();
        ROS_WARN("(Re)Initiating dead reckoning");
    }

    ROS_WARN_THROTTLE(1, "Dead reckoning");

    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    //sensor_msgs::ImuConstPtr msgorient = nullptr;

    double t_last = -1;
    for (const auto &pt : imuBufferCurr) {
        //if (msgorient == nullptr) msgorient = pt;

        double dt = 1. / 200.;
        if (t_last != -1) dt = pt->header.stamp.toSec() - t_last;

        Eigen::Quaterniond orientation;
        tf::quaternionMsgToEigen(poseManager->orientation_last, orientation);
        Eigen::Vector3d aBiasCorrected = Eigen::Vector3d(
                pt->linear_acceleration.z,
                pt->linear_acceleration.x,
                pt->linear_acceleration.y + 9.81
        );

        velocity += (orientation * aBiasCorrected) * dt;
        position += velocity * dt;

        t_last = pt->header.stamp.toSec();
    }
    imuBuffer.clear();

    /*tf2::Quaternion quat_est_rot;
    tf2::fromMsg(msgorient->orientation, quat_est_rot);
    tf2::Matrix3x3 est_rot(quat_est_rot);
    double r, p, yaw_est;
    est_rot.getRPY(r, p, yaw_est, 1);
    auto dr_yaw = (float) yaw_est;*/

    if (state == IDLE) {
        //yaw_offset = poseManager->yaw_mag_init - dr_yaw;
        state = DEAD_RECKONING;
    } else if (state == SLAM_TRACKING) {
        state = DEAD_RECKONING;
    }

    auto s = (float) sin(yaw_offset);
    auto c = (float) cos(yaw_offset);
    px += (float) position.y();//c * (float) pos_change.x() - s * (float) pos_change.z();
    py += (float) position.x();//s * (float) pos_change.x() + c * (float) pos_change.z();

    //ROS_WARN("yaw diff: %f", wpi((poseManager->yaw_mag_curr - dr_yaw)) * 180 / M_PI);

    //p_yaw = dr_yaw;

    poseManager->publishData(px, py);
}

void Fusion::imuDataCallback(const sensor_msgs::Imu::ConstPtr &imu_msg) {
    imuMutex.lock();
    imuBuffer.push_back(imu_msg);
    imuMutex.unlock();
}
