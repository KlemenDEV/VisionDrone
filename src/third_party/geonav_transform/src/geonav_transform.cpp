#include "geonav_transform/geonav_transform.h"
#include "geonav_transform/navsat_conversions.h"
#include "geonav_transform/geonav_utilities.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>

namespace GeonavTransform {
    GeonavTransform::GeonavTransform() :
            zero_altitude_(true),
            utm_frame_id_("utm"),
            odom_frame_id_("odom"),
            base_link_frame_id_("base_link"),
            utm_zone_(""),
            tf_listener_(tf_buffer_) {
        // Initialize transforms
        transform_odom2base_ = tf2::Transform(tf2::Transform::getIdentity());
        transform_odom2base_inverse_ = tf2::Transform(tf2::Transform::getIdentity());
        transform_utm2odom_ = tf2::Transform(tf2::Transform::getIdentity());
        transform_utm2odom_inverse_ = tf2::Transform(tf2::Transform::getIdentity());

        quat_orientation.setRPY(0, 0, -1.5707);
        //quat_orientation.setRPY(0, 0, 0);
    }

    GeonavTransform::~GeonavTransform() = default;

    void GeonavTransform::run() {

        double frequency = 4.;

        ros::NodeHandle nh;
        ros::NodeHandle nh_priv("~");

        nav_update_time_ = ros::Time::now();

        // Load ROS parameters
        nh_priv.param("frequency", frequency, 4.);
        nh_priv.param("zero_altitude", zero_altitude_, false);
        nh_priv.param<std::string>("base_link_frame_id", base_link_frame_id_, "base_link");
        nh_priv.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
        nh_priv.param<std::string>("utm_frame_id", utm_frame_id_, "utm");

        // Setup transforms and messages
        nav_in_odom_.header.frame_id = odom_frame_id_;
        nav_in_odom_.header.seq = 0;

        transform_msg_utm2odom_.header.frame_id = utm_frame_id_;
        transform_msg_utm2odom_.child_frame_id = odom_frame_id_;
        transform_msg_utm2odom_.header.seq = 0;

        transform_msg_odom2base_.header.frame_id = odom_frame_id_;
        transform_msg_odom2base_.child_frame_id = base_link_frame_id_;
        transform_msg_odom2base_.header.seq = 0;

        std::string topic_pub_odom;
        nh_priv.param<std::string>("pub_odom", topic_pub_odom, "geonav_odom");
        odom_pub_ = nh.advertise<geometry_msgs::PoseStamped>(topic_pub_odom, 5);

        std::string topic_sub_fix;
        nh_priv.param<std::string>("sub_fix", topic_sub_fix, "nav_fix");
        ros::Subscriber odom_sub = nh.subscribe(topic_sub_fix, 1, &GeonavTransform::navOdomCallback, this);

        ros::Subscriber imuorient_sub = nh.subscribe<sensor_msgs::Imu>("/imu/9dof", 1,
                                                                       &GeonavTransform::imuDataCallback, this);

        // Subscribe to the messages and services we need
        ros::ServiceServer datum_srv = nh.advertiseService("datum", &GeonavTransform::datumCallback, this);

        ros::Rate rate(frequency);
        while (ros::ok()) {
            ros::spinOnce();
            broadcastTf();
            rate.sleep();
        } // end of Loop
    } // end of ::run()

    void GeonavTransform::broadcastTf(void) {
        transform_msg_odom2base_.header.stamp = ros::Time::now();
        transform_msg_odom2base_.header.seq++;
        transform_msg_odom2base_.transform = tf2::toMsg(transform_odom2base_);
        tf_broadcaster_.sendTransform(transform_msg_odom2base_);
    }

    bool GeonavTransform::setDatum(double lat, double lon, double alt,
                                   tf2::Quaternion q) {
        double utm_x = 0;
        double utm_y = 0;
        NavsatConversions::LLtoUTM(lat, lon, utm_y, utm_x, utm_zone_);

        ROS_INFO_STREAM("Datum (latitude, longitude, altitude) is ("
                                << std::fixed << lat << ", "
                                << lon << ", " << alt << ")");
        ROS_INFO_STREAM("Datum UTM Zone is: " << utm_zone_);
        ROS_INFO_STREAM("Datum UTM coordinate is ("
                                << std::fixed << utm_x << ", " << utm_y << ")");


        // Set the transform utm->odom
        transform_utm2odom_.setOrigin(tf2::Vector3(utm_x, utm_y, alt));
        transform_utm2odom_.setRotation(q);
        transform_utm2odom_inverse_ = transform_utm2odom_.inverse();
        // Convert quaternion to RPY - to double check and diplay
        tf2::Matrix3x3 mat(q);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        ROS_INFO_STREAM("Datum orientation roll, pitch, yaw is ("
                                << roll << ", " << pitch << ", " << yaw << ")");


        //ROS_INFO_STREAM("Transform utm -> odom is: " << transform_utm2odom_);

        // Send out static UTM transform - frames are specified in ::run()
        transform_msg_utm2odom_.header.stamp = ros::Time::now();
        transform_msg_utm2odom_.header.seq++;
        transform_msg_utm2odom_.transform = tf2::toMsg(transform_utm2odom_);
        transform_msg_utm2odom_.transform.translation.z = (zero_altitude_ ? 0.0
                                                                          : transform_msg_utm2odom_.transform.translation.z);
        utm_broadcaster_.sendTransform(transform_msg_utm2odom_);

        datum_set = true;

        return true;
    } // end setDatum

    void GeonavTransform::navOdomCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
        if (!datum_set)
            return;

        nav_update_time_ = ros::Time::now();

        double utmX = 0;
        double utmY = 0;
        std::string utm_zone_tmp;
        NavsatConversions::LLtoUTM(msg->latitude, msg->longitude, utmY, utmX, utm_zone_tmp);

        // For now the 'nav' frame is that same as the 'base_link' frame
        transform_utm2nav_.setOrigin(tf2::Vector3(utmX, utmY, msg->altitude));

        tf2::Quaternion quat_orientation_last;
        tf2::convert(orientation_last, quat_orientation_last);
        transform_utm2nav_.setRotation(quat_orientation_last);

        transform_utm2nav_inverse_ = transform_utm2nav_.inverse();

        // Calculate Nav in odom frame
        // Note the 'base' and 'nav' frames are the same for now
        // odom2base = odom2nav = odom2utm * utm2nav
        transform_odom2base_.mult(transform_utm2odom_inverse_, transform_utm2nav_);

        tf2::Vector3 origin = transform_odom2base_.getOrigin();
        double oldx = origin.getX();
        origin.setX(origin.getY());
        origin.setY(-oldx);
        transform_odom2base_.setOrigin(origin);

        // Publish Nav odometry in odom frame - note frames are set in ::run()
        nav_in_odom_.header.stamp = nav_update_time_;
        nav_in_odom_.header.seq++;

        // Position from transform
        tf2::toMsg(transform_odom2base_, nav_in_odom_.pose);
        nav_in_odom_.pose.position.z = nav_in_odom_.pose.position.z;

        //double oldX = nav_in_odom_.pose.position.x;
        //nav_in_odom_.pose.position.x = -nav_in_odom_.pose.position.y;
        //nav_in_odom_.pose.position.y = oldX;

        odom_pub_.publish(nav_in_odom_);
    }  // navOdomCallback

    void GeonavTransform::imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg) {
        orientation_last = msg->orientation;
    }

    bool GeonavTransform::datumCallback(orb_slam3::SetDatum::Request &request,
                                        orb_slam3::SetDatum::Response &) {
        tf2::Quaternion quat_tf;
        tf2::convert(orientation_last, quat_tf);

        setDatum(request.geo_pose.position.latitude, request.geo_pose.position.longitude,
                 request.geo_pose.position.altitude, quat_tf);

        return true;
    }

} // namespace GeonavTransform

