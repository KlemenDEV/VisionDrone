#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <std_msgs/Float64.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geographic_msgs/GeoPose.h>
#include <geographic_msgs/GeoPoint.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>

#include <chrono>
#include <thread>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::Imu, sensor_msgs::Imu> syncPolicy;
typedef message_filters::Synchronizer <syncPolicy> syncer;

ros::Publisher publisher_velocity;

std::shared_ptr <syncer> syncptr;

Eigen::Vector3d gravity = Eigen::Vector3d(0, 0, 9.81);

double t_last = -1;

int counter = 0;

double roll, pitch, yaw;

double vx = 0, vy = 0;

void imuDataCallback(const sensor_msgs::Imu::ConstPtr &imu_msg, const sensor_msgs::Imu::ConstPtr &imu_msg2) {
    if (t_last == -1) {
        t_last = imu_msg2->header.stamp.toSec();
        return;
    }

    double dt = imu_msg2->header.stamp.toSec() - t_last;

    tf2::Quaternion quat_est_rot;
    tf2::fromMsg(imu_msg->orientation, quat_est_rot);
    tf2::Matrix3x3 est_rot(quat_est_rot);
    est_rot.getRPY(roll, pitch, yaw);

    double ax = cos(pitch) * -(imu_msg2->linear_acceleration.z - 0.2566709816455841);
    double ay = cos(roll) * -(imu_msg2->linear_acceleration.x - 0.03285804018378258);

    vx += ax * dt;
    vy += ay * dt;

    if (counter % 10 == 0) {
        geometry_msgs::TwistWithCovarianceStamped velocitymsg;
        velocitymsg.header.frame_id = "uav_velocity";
        velocitymsg.header.stamp = imu_msg2->header.stamp;
        velocitymsg.twist.twist.linear.x = vx;
        velocitymsg.twist.twist.linear.y = vy;
        publisher_velocity.publish(velocitymsg);
    }

    counter++;
    t_last = imu_msg->header.stamp.toSec();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "dead_reckoning");

    ros::NodeHandle nh;

    message_filters::Subscriber <sensor_msgs::Imu> imu1;
    message_filters::Subscriber <sensor_msgs::Imu> imu2;

    imu1.subscribe(nh, "/imu/9dof", 15);
    imu2.subscribe(nh, "/camera/imu", 15);
    syncptr.reset(new syncer(syncPolicy(15), imu1, imu2));
    syncptr->registerCallback(boost::bind(imuDataCallback, _1, _2));

    publisher_velocity = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/estimate/velocity_out", 1);

    ros::spin();
    ros::shutdown();

    return 0;
}