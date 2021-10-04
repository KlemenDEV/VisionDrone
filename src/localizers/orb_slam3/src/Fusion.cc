#include "Fusion.h"

Fusion::Fusion(ros::NodeHandle *nh) {
    publisher_velocity = nh->advertise<geometry_msgs::TwistWithCovarianceStamped>("/orb_slam3/velocity_out", 1);
}

void Fusion::dataSLAM(ORB_SLAM3::System *mpSLAM, const cv::Mat &Tcw) {
    if (Tcw.rows == 4 && Tcw.cols == 4) { // valid data
        float slam_x = Tcw.at<float>(0, 3);
        float slam_y = Tcw.at<float>(1, 3);

        double time_now = ros::Time::now().toSec();

        geometry_msgs::TwistWithCovarianceStamped velocitymsg;
        velocitymsg.header.frame_id = "uav_velocity";
        velocitymsg.header.stamp = ros::Time::now();

        if (mpSLAM->GetTimeFromIMUInit() > 0 && mpSLAM->mpTracker->mState == ORB_SLAM3::Tracking::OK) {
            if (t_last == -1) {
                t_last = time_now;
                return;
            }

            double dt = time_now - t_last;
            double vx = (slam_x - slam_ox) / dt;
            double vy = (slam_y - slam_oy) / dt;

            velocitymsg.twist.twist.linear.x = vx * -30.0;
            velocitymsg.twist.twist.linear.y = vy * -30.0;
            publisher_velocity.publish(velocitymsg);

            t_last = time_now;
        } else {
            t_last = -1; // reset velocity

            velocitymsg.twist.covariance[0] = NAN;
            velocitymsg.twist.twist.linear.x = 0;
            velocitymsg.twist.twist.linear.y = 0;
            publisher_velocity.publish(velocitymsg);
        }

        slam_ox = slam_x;
        slam_oy = slam_y;
    }
}