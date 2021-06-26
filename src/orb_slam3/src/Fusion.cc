#include "Fusion.h"

Fusion::Fusion(ros::NodeHandle *nh) {
    this->point_pub = nh->advertise<geometry_msgs::PoseStamped>("/orbslam3/pose", 1);

    this->gps_pub = nh->advertise<sensor_msgs::NavSatFix>("/ublox/fix_tracking", 1);

    // IMU orientation (for initial pose)
    imuorient_sub = nh->subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1, &Fusion::imuDataCallback, this);

    // Height estimator
    height_sub = nh->subscribe<std_msgs::Float64>("/drone/height_estimate", 1, &Fusion::heightCallback, this);

    // GPS ground truth
    gpsfix_sub = nh->subscribe<sensor_msgs::NavSatFix>("/ublox/fix", 1, &Fusion::gpsDataCallback, this);

    set_datum_client = nh->serviceClient<robot_localization::SetDatum>("/datum");

    // pre-init states
    velocity = Eigen::Vector3f::Zero();
    orientation = Eigen::Quaternionf::Identity();

    gravity(0) = 0.0;
    gravity(1) = 9.81;
    gravity(2) = 0.0;

    aBias = Eigen::Vector3f(0.03285804018378258, -0.12473473697900772, 0.2566709816455841);
    gBias = Eigen::Vector3f(-0.00011866784916492179, 6.184831363498233e-06, 2.998005766130518e-05);
}

void Fusion::imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    orientation_last = msg->orientation;
}

void Fusion::gpsDataCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    gps_last = msg;

    if (tracking_started) this->gps_pub.publish(msg);
}

void Fusion::heightCallback(const std_msgs::Float64::ConstPtr &msg) {
    height_pre_last = height_last;
    height_last = (float) msg->data;
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
        ROS_WARN("GLOBAL GPS HOME LOCATION (DATUM) set and propagated to other services");
        ROS_WARN("DATUM: lat: %f, lon: %f, alt: %f", datumPosition.latitude, datumPosition.longitude,
                 datumPosition.altitude);
    } else {
        ROS_ERROR("FAILED TO PROPAGATE GLOBAL GPS HOME LOCATION (DATUM)");
    }
}

void Fusion::publishData() {
    if (tracking_started) {
        geometry_msgs::PoseStamped posest;
        geometry_msgs::Pose pose;
        geometry_msgs::Point pt;

        float s = sin(yaw_corr);
        float c = cos(yaw_corr);
        pt.x = c * px - s * py;
        pt.y = s * px + c * py;
        pt.z = pz + height_last;

        pose.position = pt;
        posest.pose = pose;
        posest.header.stamp = ros::Time::now();
        posest.header.frame_id = "map";
        this->point_pub.publish(posest);
    }
}

void Fusion::makeQuaternionFromVector(Eigen::Vector3f &inVec, Eigen::Quaternionf &outQuat) {
    float phi = inVec.norm();
    Eigen::Vector3f u = inVec / phi;

    outQuat.vec() = u * (float) sin(phi / 2.0);
    outQuat.w() = (float) cos(phi / 2.0);
}

void Fusion::deadReckoning(const vector<ORB_SLAM3::IMU::Point> &vImuMeas) {
    if (!dead_reckoning) {
        velocity = Eigen::Vector3f::Zero();
        orientation = Eigen::Quaternionf::Identity();
        dead_reckoning = true;
        ROS_WARN("(Re)Initiating dead reckoning");
    }

    ROS_WARN_THROTTLE(1, "Dead reckoning");

    Eigen::Vector3f pos_change = Eigen::Vector3f::Zero();

    double t_last = -1;
    for (const auto &pt : vImuMeas) {
        double delT = 1. / 200.;
        if (t_last != -1)
            delT = pt.t - t_last;

        Eigen::Matrix3f R = orientation.toRotationMatrix();
        Eigen::Vector3f aBiasCorrected = Eigen::Vector3f(pt.a.x, pt.a.y, pt.a.z) - aBias;
        Eigen::Vector3f gBiasCorrected = (Eigen::Vector3f(pt.w.x, pt.w.y, pt.w.z) - gBias) * delT;

        pos_change += velocity * delT + 0.5 * (R * aBiasCorrected + gravity) * delT * delT;
        velocity += (R * aBiasCorrected + gravity) * delT;

        Eigen::Quaternionf Q;
        makeQuaternionFromVector(gBiasCorrected, Q);
        orientation = orientation * Q;

        t_last = pt.t;
    }

    float chg = sqrt(((float) pos_change.x() * (float) pos_change.x())
                     + ((float) pos_change.z() * (float) pos_change.z()));
    tf2::Quaternion quat_gps_rot;
    tf2::fromMsg(orientation_last, quat_gps_rot);
    tf2::Matrix3x3 gps_tf2_rot(quat_gps_rot);
    double roll, pitch, yaw;
    gps_tf2_rot.getRPY(roll, pitch, yaw, 1);

    odx = (float) (chg * cos((float) wpi(yaw - yaw_corr)));
    ody = (float) (chg * sin((float) wpi(yaw - yaw_corr)));

    cout << vImuMeas.size() << " : " << chg << endl;

    px += odx;
    py += ody;

    this->publishData();
}

void Fusion::dataSLAM(ORB_SLAM3::System *mpSLAM, const cv::Mat &Tcw, vector<ORB_SLAM3::IMU::Point> &vImuMeas) {
    if (Tcw.rows == 4 && Tcw.cols == 4) { // valid data
        cv::Mat Twc(4, 4, CV_32F);
        cv::invert(Tcw, Twc);

        float x = Twc.at<float>(0, 3);
        float y = Twc.at<float>(1, 3);

        tf2::Matrix3x3 tf2_rot(Twc.at<double>(0, 0), Twc.at<double>(0, 1), Twc.at<double>(0, 2),
                               Twc.at<double>(1, 0), Twc.at<double>(1, 1), Twc.at<double>(1, 2),
                               Twc.at<double>(2, 0), Twc.at<double>(2, 1), Twc.at<double>(2, 2));


        double roll2, pitch2, yaw2;
        tf2_rot.getRPY(roll2, pitch2, yaw2, 1);

        if (!tracking_started) {
            tf2::Quaternion quat_gps_rot;
            tf2::fromMsg(orientation_last, quat_gps_rot);
            tf2::Matrix3x3 gps_tf2_rot(quat_gps_rot);

            double roll, pitch, yaw;
            gps_tf2_rot.getRPY(roll, pitch, yaw, 1);

            yaw_corr += (float) wpi(wpi(yaw) - wpi(yaw2));

            avgcounter++;
            if (avgcounter == 5) {
                yaw_corr /= (float) avgcounter;

                setGPSDatum();
                pz -= height_last;

                tracking_started = true;
                ROS_WARN("Tracking started");
            }
        } else if (!dead_reckoning) {
            //orientation = Eigen::AngleAxisf((float) roll2, Eigen::Vector3f::UnitX())
            //              * Eigen::AngleAxisf((float) pitch2, Eigen::Vector3f::UnitY())
            //              * Eigen::AngleAxisf((float) yaw2, Eigen::Vector3f::UnitZ());

            if (mpSLAM->GetTimeFromIMUInit() <= 0 || mpSLAM->mpTracker->mState != ORB_SLAM3::Tracking::OK) {
                this->deadReckoning(vImuMeas);
                return;
            }

            float dx = x - ox;
            float dy = y - oy;

            // filter out big jumps (eg. due to map re-alignments)
            if (sqrt(dx * dx + dy * dy) > 25) {
                ROS_WARN("Large jump detected! Switching to dead reckoning");
                this->deadReckoning(vImuMeas);
                return;
            }

            ROS_WARN_THROTTLE(1, "SLAM");

            float chg = sqrt(dx*dx + dy*dy);
            tf2::Quaternion quat_gps_rot;
            tf2::fromMsg(orientation_last, quat_gps_rot);
            tf2::Matrix3x3 gps_tf2_rot(quat_gps_rot);
            double roll, pitch, yaw;
            gps_tf2_rot.getRPY(roll, pitch, yaw, 1);
            px += (float) (chg * cos((float) wpi(yaw - yaw_corr)));
            py += (float) (chg * sin((float) wpi(yaw - yaw_corr)));

            //px += dx;
            //py += dy;

            odx = dx;
            ody = dy;

            this->publishData();
        } else if (mpSLAM->GetTimeFromIMUInit() > 0 && mpSLAM->mpTracker->mState == ORB_SLAM3::Tracking::OK) {
            dead_reckoning = false;
            ROS_WARN("Back to SLAM");
        } else {
            this->deadReckoning(vImuMeas);
            return;
        }

        ox = x;
        oy = y;
    }
}
