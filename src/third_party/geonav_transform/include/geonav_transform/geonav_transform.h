/* 

Copyright (c) 2017, Brian Bingham
All rights reserved

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

#ifndef GEONAV_TRANSFORM_GEONAV_TRANSFORM_H
#define GEONAV_TRANSFORM_GEONAV_TRANSFORM_H

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>

#include <string>
#include <orb_slam3/SetDatum.h>

#include <geometry_msgs/PoseStamped.h>

namespace GeonavTransform {

    class GeonavTransform {
    public:
        //! @brief Constructor
        //!
        GeonavTransform();

        //! @brief Destructor
        //!
        ~GeonavTransform();

        //! @brief Main run loop
        //!
        void run();

    private:

        //! @brief Sets datum values
        //! yaw is ENU
        //!
        bool setDatum(double lat, double lon, double alt, tf2::Quaternion q);


        //! @brief Callback for the geo nav odom data
        //! @param[in] msg The odometry message to process
        //!
        void navOdomCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);

        //! @brief Sends transform
        void broadcastTf();

        bool datumCallback(orb_slam3::SetDatum::Request& request,
                           orb_slam3::SetDatum::Response&);

        void imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg);

        geometry_msgs::Quaternion orientation_last;

        //! @brief Frame ID of the robot's body frame
        //!
        std::string base_link_frame_id_;

        //! @brief Frame ID of the "odom" frame
        //!
        std::string odom_frame_id_;

        //! @brief Frame ID of the "odom" frame
        //!
        std::string utm_frame_id_;

        //! @brief Whether or not we broadcast the utm->odom transform
        //!
        bool broadcast_utm2odom_transform_;

        //! @brief Whether or not we broadcast the odom->base_link transform
        //!
        bool broadcast_odom2base_transform_;

        //! @brief The frame_id of the NAV message (specifies mounting location)
        //!
        std::string nav_frame_id_;

        //! @brief Timestamp of the latest good NAV message
        //!
        //! We assign this value to the timestamp of the odometry message that we output
        //!
        ros::Time nav_update_time_;

        //! @brief Latest NAV data, stored as UTM coords
        //!
        tf2::Transform transform_utm2nav_;
        tf2::Transform transform_utm2nav_inverse_;

        //! @brief Transform buffer for managing coordinate transforms
        //!
        tf2_ros::Buffer tf_buffer_;

        //! @brief Transform listener for receiving transforms
        //!
        tf2_ros::TransformListener tf_listener_;

        //! @brief Whether or not we've computed a good heading
        //!
        bool transform_good_;

        //! @brief Used for publishing the static utm->odom transform
        //!
        tf2_ros::StaticTransformBroadcaster utm_broadcaster_;

        //! @brief Used for publishing the static utm->odom transform
        //!
        tf2_ros::TransformBroadcaster tf_broadcaster_;

        //! @brief Holds the UTM->odom transform
        //!
        tf2::Transform transform_utm2odom_;
        tf2::Transform transform_utm2odom_inverse_;
        //! @brief Message
        geometry_msgs::TransformStamped transform_msg_utm2odom_;
        geometry_msgs::PoseStamped nav_in_odom_;

        //! @brief Holds the odom->base transform
        //!
        tf2::Transform transform_odom2base_;
        tf2::Transform transform_odom2base_inverse_;
        //! @brief Messages
        geometry_msgs::TransformStamped transform_msg_odom2base_;
        geometry_msgs::PoseStamped nav_in_utm_;

        //! @brief UTM zone as determined after transforming GPS message
        //!
        std::string utm_zone_;

        //! @brief Whether or not to report 0 altitude
        //!
        //! If this parameter is true, we always report 0 for the altitude of the converted GPS odometry message.
        //!
        bool zero_altitude_;

        //! @brief Publisher for Nav relative to odom (datum) frame
        ros::Publisher odom_pub_;
        //! @brief Publisher for Nav Odometry relative to utm frame
        ros::Publisher utm_pub_;

        bool datum_set;
    };

}  // namespace GeonavTransform

#endif  // GEONAV_TRANSFORM_NAVSAT_TRANSFORM_H
