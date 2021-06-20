#include <ros/ros.h>

#include <sensor_msgs/FluidPressure.h>
#include <std_msgs/Float64.h>

ros::Publisher height_pub;

bool init = false;
double pressure_int = 0;

void baroCallback(const sensor_msgs::FluidPressure::ConstPtr &msg) {
    if (!init) {
        pressure_int = msg->fluid_pressure;
        init = true;
    } else {

    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "height_estimation_node");
    ros::NodeHandle nh;

    ros::Subscriber baro_sub = nh.subscribe<sensor_msgs::FluidPressure>
            ("/mavros/imu/diff_pressure", 1, baroCallback);

    height_pub = nh.advertise<std_msgs::Float64>("/drone/height_estimate", 1);

    ros::spin();
    ros::shutdown();

    return 0;
}
