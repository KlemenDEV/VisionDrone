#include <Fusion.h>

void Fusion::imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    orientation_last = msg->orientation;
}

void Fusion::gpsDataCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    gps_last = msg;
}

Fusion::Fusion(ros::NodeHandle *nh) {
    this->point_pub = nh->advertise<geometry_msgs::PoseStamped>("/orbslam3/pose_direct", 5);
    this->point_pub2 = nh->advertise<geometry_msgs::PoseStamped>("/orbslam3/pose_rel", 5);

    // IMU orientation (for initial pose)
    imuorient_sub = nh->subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1, &Fusion::imuDataCallback, this);

    gpsfix_sub = nh->subscribe<sensor_msgs::NavSatFix>("/ublox/fix", 1, &Fusion::gpsDataCallback, this);

    set_datum_client = nh->serviceClient<robot_localization::SetDatum>("/datum");
}

void Fusion::setGPSDatum() {
    geographic_msgs::GeoPoint datumPosition;
    datumPosition.latitude = gps_last->latitude;
    datumPosition.longitude = gps_last->longitude;
    datumPosition.altitude = gps_last->altitude;

    geographic_msgs::GeoPose datumPose;
    datumPose.position = datumPosition;
    datumPose.orientation = orientation_last;

    robot_localization::SetDatum setDatum;
    setDatum.request.geo_pose = datumPose;

    if (set_datum_client.call(setDatum)) {
        ROS_INFO("GLOBAL GPS HOME LOCATION (DATUM) set and propagated to other services");
        ROS_INFO("DATUM: lat: %f, lon: %f, alt: %f", datumPosition.latitude, datumPosition.longitude,
                 datumPosition.altitude);
    } else {
        ROS_ERROR("FAILED TO PROPAGATE GLOBAL GPS HOME LOCATION (DATUM)");
    }
}

void Fusion::dataSLAM(ORB_SLAM3::System *mpSLAM, const cv::Mat &Tcw) {
    if (Tcw.rows == 4 && Tcw.cols == 4) { // valid data
        if (mpSLAM->GetTimeFromIMUInit() <= 0 || mpSLAM->mpTracker->mState != ORB_SLAM3::Tracking::OK) {
            reset = true;
            return;
        }

        cv::Mat Twc(4, 4, CV_32F);
        cv::invert(Tcw, Twc);

        float x = Twc.at<float>(0, 3);
        float y = Twc.at<float>(1, 3);
        float z = Twc.at<float>(2, 3);

        geometry_msgs::PoseStamped posest;
        geometry_msgs::Pose pose;
        geometry_msgs::Point pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        pose.position = pt;
        posest.pose = pose;
        posest.header.stamp = ros::Time::now();
        posest.header.frame_id = "map";
        this->point_pub.publish(posest);

        float dx = x - ox;
        float dy = y - oy;
        float dz = z - oz;

        float dd = sqrt(dx * dx + dy * dy + dz * dz);
        if (dd > 4) {
            // TODO: dead reckoning after a while?
            dx = odx;
            dy = ody;
            dz = odz;
        }

        if (!reset) {
            px += dx;
            py += dy;
            pz += dz;
        } else {
            reset = false;
        }

        ox = x;
        oy = y;
        oz = z;

        odx = dx;
        ody = dy;
        odz = dz;

        geometry_msgs::PoseStamped posest2;
        geometry_msgs::Pose pose2;
        geometry_msgs::Point pt2;
        pt2.x = px;
        pt2.y = py;
        pt2.z = pz;
        pose2.position = pt2;
        posest2.pose = pose2;
        posest2.header.stamp = ros::Time::now();
        posest2.header.frame_id = "map";
        this->point_pub2.publish(posest2);

        if (!tracking_started) {
            setGPSDatum();
            tracking_started = true;
        }
    }
}