#include <ros/ros.h>

#include <sensor_msgs/FluidPressure.h>
#include <std_msgs/Float64.h>

#define P0 101200

using namespace std;

ros::Publisher height_pub;

int avgcounter = 0;
float h_init = 0;

void baroCallback(const sensor_msgs::FluidPressure::ConstPtr &msg) {
    float h_curr = (float) (44330 * (1 - pow(msg->fluid_pressure / P0, 1 / 5.255)));

    if (avgcounter == 5) {
        h_init /= (float) avgcounter;
        avgcounter++;
        cout << "Baro init height:" << h_curr << " m" << endl;
    }

    if (avgcounter >= 5) {
        std_msgs::Float64 fmsg;
        fmsg.data = h_curr - h_init;
        height_pub.publish(fmsg);
    } else {
        avgcounter++;
        h_init += h_curr;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "height_estimation_node");
    ros::NodeHandle nh;

    ros::Subscriber baro_sub = nh.subscribe<sensor_msgs::FluidPressure>
            ("/mavros/imu/static_pressure", 5, baroCallback);

    height_pub = nh.advertise<std_msgs::Float64>("/drone/height_estimate", 5);

    ros::spin();
    ros::shutdown();

    return 0;
}
