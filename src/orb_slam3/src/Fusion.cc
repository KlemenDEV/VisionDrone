#include "Fusion.h"

Fusion::Fusion(ros::NodeHandle *nh) {
    poseManager = new PoseManager(nh);

    // pre-init states
    velocity = Eigen::Vector3f::Zero();
    orientation = Eigen::Quaternionf::Identity();
    gravity(0) = 0.0;
    gravity(1) = 9.81;
    gravity(2) = 0.0;
    aBias = Eigen::Vector3f(0.03285804018378258, -0.12473473697900772, 0.2566709816455841);
    gBias = Eigen::Vector3f(-0.00011866784916492179, 6.184831363498233e-06, 2.998005766130518e-05);

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

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
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
                state = SLAM_TRACKING;
            }

            slam_ox = slam_x;
            slam_oy = slam_y;
            ROS_WARN("Back to SLAM");*/
        }
    }
}

void Fusion::deadReckoning(const vector<sensor_msgs::ImuConstPtr> &imuBufferCurr) {
    if (imuBufferCurr.empty()) return;

    if (state == IDLE) {
        if (warmupCounter < 2000) {
            warmupCounter++;
        } else if (warmupCounter == 2000) {
            state = WARMUP_COMPLETE;
        }
        return;
    }

    if (state != DEAD_RECKONING) {
        velocity = Eigen::Vector3f::Zero();
        orientation = Eigen::Quaternionf::Identity();
        ROS_WARN("(Re)Initiating dead reckoning");
    }

    ROS_WARN_THROTTLE(1, "Dead reckoning");

    Eigen::Vector3f pos_change = Eigen::Vector3f::Zero();

    double t_last = -1;
    for (const auto &pt : imuBufferCurr) {
        double delT = 1. / 200.;
        if (t_last != -1) delT = pt->header.stamp.toSec() - t_last;

        Eigen::Matrix3f R = orientation.toRotationMatrix();
        Eigen::Vector3f aBiasCorrected = Eigen::Vector3f(
                (float) pt->linear_acceleration.x,
                (float) pt->linear_acceleration.y,
                (float) pt->linear_acceleration.z
        ) - aBias;
        aBiasCorrected = Eigen::Vector3f(-aBiasCorrected.x(), -aBiasCorrected.y(), aBiasCorrected.z());

        Eigen::Vector3f gBiasCorrected = Eigen::Vector3f(
                (float) pt->angular_velocity.x,
                (float) pt->angular_velocity.y,
                (float) pt->angular_velocity.z
        ) - gBias;
        gBiasCorrected = Eigen::Vector3f(-gBiasCorrected.x(), -gBiasCorrected.y(), gBiasCorrected.z());

        pos_change += velocity * delT + 0.5 * (R * aBiasCorrected - gravity) * delT * delT;
        velocity += (R * aBiasCorrected - gravity) * delT;

        Eigen::Quaternionf Q;
        gBiasCorrected = gBiasCorrected * delT;
        makeQuaternionFromVector(gBiasCorrected, Q);
        orientation = orientation * Q;

        t_last = pt->header.stamp.toSec();
    }
    imuBuffer.clear();

    float dr_yaw = orientation.toRotationMatrix().eulerAngles(0, 1, 2)[2];

    if (state == WARMUP_COMPLETE) {
        yaw_offset = dr_yaw - poseManager->yaw_mag_init;
        state = DEAD_RECKONING;
    } else if (state == SLAM_TRACKING) {
        state = DEAD_RECKONING;
    }

    auto s = (float) sin(yaw_offset);
    auto c = (float) cos(yaw_offset);
    px += c * pos_change.x() - s * pos_change.z();
    py += s * pos_change.x() + c * pos_change.z();
    p_yaw = dr_yaw;

    poseManager->publishData(px, py);
}

void Fusion::addIMUMeasurement(const sensor_msgs::ImuConstPtr &imu_msg) {
    imuMutex.lock();
    imuBuffer.push_back(imu_msg);
    imuMutex.unlock();
}

void Fusion::makeQuaternionFromVector(Eigen::Vector3f &inVec, Eigen::Quaternionf &outQuat) {
    float phi = inVec.norm();
    Eigen::Vector3f u = inVec / phi;

    outQuat.vec() = u * (float) sin(phi / 2.0);
    outQuat.w() = (float) cos(phi / 2.0);
}
