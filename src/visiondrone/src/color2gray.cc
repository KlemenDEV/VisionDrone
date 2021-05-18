#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

image_transport::Publisher image_pub;

void imageCb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    image_pub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "color2gray");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    std::string topic_in;
    nh.getParam("in", topic_in);
    std::string topic_out;
    nh.getParam("out", topic_out);

    image_transport::ImageTransport it(n);
    image_transport::Subscriber image_sub = it.subscribe(topic_in, 1, imageCb);
    image_pub = it.advertise(topic_out, 1);

    ros::spin();
    
    return 0;
}