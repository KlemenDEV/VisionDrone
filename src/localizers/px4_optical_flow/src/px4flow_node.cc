#include <ros/ros.h>

#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/Quaternion.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

#include "flow_opencv.hpp"
#include "flow_px4.hpp"

OpticalFlowPX4 flowpx4(477.78586954352323, 480.6678820118329,
                       -1, // image rate
                       640, 480
);

OpticalFlowOpenCV flow(477.78586954352323, 480.6678820118329,
                       -1, // image rate
                       640, 480,
                       200, // default 20
                       1.645f //90% confidence interval
);

ros::Publisher publisher_velocity;

double height_last = 0;

double roll = 0, pitch = 0, yaw = 0;

double wx, wy;

double vx, vy;

int init_counter = 0;

bool use_px4;

void heightCallback(const std_msgs::Float64::ConstPtr &msg) {
    height_last = msg->data;
}

void imuDataCallback(const sensor_msgs::Imu::ConstPtr &imu_msg) {
    tf2::Quaternion quat_est_rot;
    tf2::fromMsg(imu_msg->orientation, quat_est_rot);
    tf2::Matrix3x3 est_rot(quat_est_rot);
    est_rot.getRPY(roll, pitch, yaw);

    wx = imu_msg->angular_velocity.x * 0.9 + wx * 0.1;
    wy = imu_msg->angular_velocity.y * 0.9 + wy * 0.1;
}

void callbackImage(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    uint32_t usec_stamp = msg->header.stamp.toNSec() * 1e-3;

    float cx, cy;
    int dtus, qty;
    if (use_px4)
        qty = flowpx4.calcFlow(image->image.data, usec_stamp, dtus, cx, cy);
    else
        qty = flow.calcFlow(image->image.data, usec_stamp, dtus, cx, cy);

    double sensor_dist = abs(height_last / (cos(roll) * cos(pitch)));

    double nvx = sensor_dist * (wy - (double) cy / (dtus * 1e-6));
    double nvy = sensor_dist * (wx - (double) cx / (dtus * 1e-6));

    vx = nvx * 0.9 + vx * 0.1;
    vy = nvy * 0.9 + vy * 0.1;

    geometry_msgs::TwistWithCovarianceStamped velocity;
    velocity.header.frame_id = "uav_velocity";
    velocity.header.stamp = msg->header.stamp;
    velocity.twist.twist.linear.x = vx;
    velocity.twist.twist.linear.y = vy;

    if (qty < 0)
        velocity.twist.covariance[0] = NAN;
    else
        velocity.twist.covariance[0] = qty / 255.;

    publisher_velocity.publish(velocity);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "px4flow_node");
    ros::NodeHandle nh;
    ros::NodeHandle n("~");

    n.param<bool>("use_px4", use_px4, false);

    if (use_px4) {
        ROS_WARN("====== USING PX4 FLOW");
    } else {
        ROS_WARN("====== USING OPENCV FLOW");
    }

    flow.setCameraMatrix(477.78586954352323, 480.6678820118329, 322.72767560819693, 258.974159781733);
    flow.setCameraDistortion(0.11906203790630414, -0.23224501485827584, 0.002897948377514225, -0.0026544348133675866);

    ros::Subscriber sub_img = nh.subscribe("/camera/orthogonal", 1, callbackImage);
    ros::Subscriber sub_imu = nh.subscribe("/imu/9dof", 5, imuDataCallback);
    ros::Subscriber sub_height = nh.subscribe("/drone/height_estimate", 1, heightCallback);

    publisher_velocity = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/optical_flow/velocity_out", 1);

    ros::spin();
    ros::shutdown();

    return 0;
}