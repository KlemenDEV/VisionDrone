/* 

Copyright (c) 2017, Brian Bingham
All rights reserved
    <depend>robot_localization</depend>

This file is part of the geonav_transform package.

Geonav_transform is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Geonav_transform is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "geonav_transform/geonav_transform.h"
#include "geonav_transform/navsat_conversions.h"
#include "geonav_transform/geonav_utilities.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <XmlRpcException.h>

#include <string>
#include <robot_localization/SetDatum.h>

namespace GeonavTransform {
    GeonavTransform::GeonavTransform() :
    // Initialize attributes
            broadcast_utm2odom_transform_(true),
            broadcast_odom2base_transform_(true),
            nav_frame_id_(""),
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
    }

    GeonavTransform::~GeonavTransform() {
    }

    void GeonavTransform::run() {

        double frequency = 10.0;
        double delay = 0.0;

        ros::NodeHandle nh;
        ros::NodeHandle nh_priv("~");

        nav_update_time_ = ros::Time::now();

        // Load ROS parameters
        nh_priv.param("frequency", frequency, 4.);
        nh_priv.param("orientation_ned", orientation_ned_, false);

        nh_priv.param("broadcast_utm2odom_transform", broadcast_utm2odom_transform_, true);
        nh_priv.param("broadcast_odom2base_transform", broadcast_odom2base_transform_, true);
        nh_priv.param("zero_altitude", zero_altitude_, false);
        nh_priv.param<std::string>("base_link_frame_id", base_link_frame_id_, "base_link");
        nh_priv.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
        nh_priv.param<std::string>("utm_frame_id", utm_frame_id_, "utm");

        // Setup transforms and messages 
        nav_in_odom_.header.frame_id = odom_frame_id_;
        nav_in_odom_.child_frame_id = base_link_frame_id_;
        nav_in_odom_.header.seq = 0;
        nav_in_utm_.header.frame_id = utm_frame_id_;
        nav_in_utm_.child_frame_id = base_link_frame_id_;
        nav_in_utm_.header.seq = 0;
        transform_msg_utm2odom_.header.frame_id = utm_frame_id_;
        transform_msg_utm2odom_.child_frame_id = odom_frame_id_;
        transform_msg_utm2odom_.header.seq = 0;
        transform_msg_odom2base_.header.frame_id = odom_frame_id_;
        transform_msg_odom2base_.child_frame_id = base_link_frame_id_;
        transform_msg_odom2base_.header.seq = 0;

        std::string topic_pub_odom;
        nh_priv.param<std::string>("pub_odom", topic_pub_odom, "geonav_odom");
        odom_pub_ = nh.advertise<nav_msgs::Odometry>(topic_pub_odom, 10);

        std::string topic_pub_utm;
        nh_priv.param<std::string>("pub_utm", topic_pub_utm, "geonav_utm");
        utm_pub_ = nh.advertise<nav_msgs::Odometry>(topic_pub_utm, 10);

        std::string topic_sub_fix;
        nh_priv.param<std::string>("sub_fix", topic_sub_fix, "nav_fix");
        ros::Subscriber odom_sub = nh.subscribe(topic_sub_fix, 1, &GeonavTransform::navOdomCallback, this);

        // Subscribe to the messages and services we need
        ros::ServiceServer datum_srv = nh.advertiseService("datum", &GeonavTransform::datumCallback, this);

        ros::Rate rate(frequency);
        while (ros::ok()) {
            ros::spinOnce();
            broadcastTf();
            rate.sleep();
        } // end of Loop
    } // end of ::run()

    void GeonavTransform::broadcastTf() {
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

        // Convert quaternion to RPY - to double check and display
        tf2::Matrix3x3 mat(q);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        ROS_INFO_STREAM("Datum orientation roll, pitch, yaw is ("
                                << roll << ", " << pitch << ", " << yaw << ")");

        // Send out static UTM transform - frames are specified in ::run()
        transform_msg_utm2odom_.header.stamp = ros::Time::now();
        transform_msg_utm2odom_.header.seq++;
        transform_msg_utm2odom_.transform = tf2::toMsg(transform_utm2odom_);
        transform_msg_utm2odom_.transform.translation.z = (zero_altitude_ ? 0.0
                                                                          : transform_msg_utm2odom_.transform.translation.z);
        utm_broadcaster_.sendTransform(transform_msg_utm2odom_);

        datum_set = true;
    } // end setDatum

    void GeonavTransform::navOdomCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
        nav_frame_id_ = msg->header.frame_id;

        if (!datum_set)
            return;

        nav_update_time_ = ros::Time::now();

        double utmX = 0;
        double utmY = 0;
        std::string utm_zone_tmp;
        NavsatConversions::LLtoUTM(msg->latitude, msg->longitude,
                                   utmY, utmX, utm_zone_tmp);

        // For now the 'nav' frame is that same as the 'base_link' frame
        transform_utm2nav_.setOrigin(tf2::Vector3(utmX, utmY,
                                                  msg->altitude));
        transform_utm2nav_.setRotation(tf2::Quaternion::getIdentity());
        transform_utm2nav_inverse_ = transform_utm2nav_.inverse();

        // Publish Nav/Base Odometry in UTM frame - note frames are set in ::run()
        nav_in_utm_.header.stamp = nav_update_time_;
        nav_in_utm_.header.seq++;
        // Create position information using transform.
        // Convert from transform to pose message
        //tf2::toMsg(transform_utm2nav_, nav_in_utm_.pose.pose);
        tf2::Vector3 tmp;
        tmp = transform_utm2nav_.getOrigin();
        nav_in_utm_.pose.pose.position.x = tmp[0];
        nav_in_utm_.pose.pose.position.y = tmp[1];
        nav_in_utm_.pose.pose.position.z = tmp[2];

        nav_in_utm_.pose.pose.position.z = (zero_altitude_ ? 0.0 : nav_in_utm_.pose.pose.position.z);

        // Publish
        // utm_pub_.publish(nav_in_utm_);

        // Calculate Nav in odom frame
        // Note the 'base' and 'nav' frames are the same for now
        // odom2base = odom2nav = odom2utm * utm2nav
        transform_odom2base_.mult(transform_utm2odom_inverse_, transform_utm2nav_);

        // Publish Nav odometry in odom frame - note frames are set in ::run()
        nav_in_odom_.header.stamp = nav_update_time_;
        nav_in_odom_.header.seq++;

        // Position from transform
        tf2::toMsg(transform_odom2base_, nav_in_odom_.pose.pose);
        nav_in_odom_.pose.pose.position.z = (zero_altitude_ ? 0.0 : nav_in_odom_.pose.pose.position.z);

        odom_pub_.publish(nav_in_odom_);
    }  // navOdomCallback

    bool GeonavTransform::datumCallback(robot_localization::SetDatum::Request &request,
                                        robot_localization::SetDatum::Response &) {
        tf2::Quaternion quat_tf;
        tf2::convert(request.geo_pose.orientation, quat_tf);

        setDatum(request.geo_pose.position.latitude, request.geo_pose.position.longitude,
                 request.geo_pose.position.altitude, quat_tf);

        return true;
    }


} // namespace GeonavTransform

