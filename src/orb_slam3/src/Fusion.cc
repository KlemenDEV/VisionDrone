#include "Fusion.h"

void Fusion::imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    orientation_last = msg->orientation;
}

void Fusion::gpsDataCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    gps_last = msg;
}

void Fusion::heightCallback(const std_msgs::Float64::ConstPtr &msg) {
    height_last = msg->data;
}

Fusion::Fusion(ros::NodeHandle *nh) {
    this->point_pub = nh->advertise<geometry_msgs::PoseStamped>("/orbslam3/pose", 1);

    // IMU orientation (for initial pose)
    imuorient_sub = nh->subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1, &Fusion::imuDataCallback, this);

    // Height estimator
    height_sub = nh->subscribe<std_msgs::Float64>("/drone/height_estimate", 1, &Fusion::heightCallback, this);

    // GPS ground truth
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

double wpi(double x) {
    return atan2(sin(x), cos(x));
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

        if (!tracking_started) {
            tf2::Matrix3x3 tf2_rot(Twc.at<float>(0, 0), Twc.at<float>(0, 1), Twc.at<float>(0, 2),
                                   Twc.at<float>(1, 0), Twc.at<float>(1, 1), Twc.at<float>(1, 2),
                                   Twc.at<float>(2, 0), Twc.at<float>(2, 1), Twc.at<float>(2, 2));


            tf2::Quaternion quat_gps_rot;
            tf2::fromMsg(orientation_last, quat_gps_rot);
            tf2::Matrix3x3 gps_tf2_rot(quat_gps_rot);

            double roll, pitch, yaw;
            gps_tf2_rot.getRPY(roll, pitch, yaw, 1);
            double roll2, pitch2, yaw2;
            tf2_rot.getRPY(roll2, pitch2, yaw2, 1);

            yaw_corr += (float) wpi(wpi(yaw) - wpi(yaw2) - M_PI / 2 + 0.075921822 * 5);

            avgcounter++;
            if(avgcounter == 50) {
                yaw_corr /= 50.;

                setGPSDatum();

                ox = x;
                oy = y;
                oz = z;

                tracking_started = true;
                cout << "Tracking started" << endl;
            }
        } else {
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
        }

        geometry_msgs::PoseStamped posest;
        geometry_msgs::Pose pose;

        geometry_msgs::Point pt;

        float s = sin(yaw_corr);
        float c = cos(yaw_corr);
        pt.x = c * px - s * py;
        pt.y = s * px + c * py;
        pt.z = height_last;

        pose.position = pt;
        posest.pose = pose;
        posest.header.stamp = ros::Time::now();
        posest.header.frame_id = "map";
        this->point_pub.publish(posest);
    }
}