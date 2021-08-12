#include "PoseManager.h"

PoseManager::PoseManager(ros::NodeHandle *nh) {
    gps_pub = nh->advertise<sensor_msgs::NavSatFix>("/ublox/fix_tracking", 1);
    point_pub = nh->advertise<geometry_msgs::PoseStamped>("/orbslam3/pose", 1);

    // IMU EARTH orientation (for initial pose)
    imuorient_sub = nh->subscribe<sensor_msgs::Imu>("/imu/9dof", 1, &PoseManager::imuDataCallback, this);

    // GPS ground truth
    gpsfix_sub = nh->subscribe<sensor_msgs::NavSatFix>("/ublox/fix", 1, &PoseManager::gpsDataCallback, this);

    // Height estimator
    height_sub = nh->subscribe<std_msgs::Float64>("/drone/height_estimate", 1, &PoseManager::heightCallback, this);

    set_datum_client = nh->serviceClient<orb_slam3::SetDatum>("/datum");
}

#define IMU_SAMPLES 100

void PoseManager::imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    tf2::Quaternion quat_gps_rot;
    tf2::fromMsg(msg->orientation, quat_gps_rot);
    tf2::Matrix3x3 gps_tf2_rot(quat_gps_rot);
    double r, p, yaw_mag;
    gps_tf2_rot.getRPY(r, p, yaw_mag, 1);

    yaw_mag_curr = (float) yaw_mag;

    if (imuDataCount > IMU_SAMPLES && !datum_set) {
        yaw_mag_init /= (float) IMU_SAMPLES;

        ROS_WARN("Initial yaw pose: %f deg", yaw_mag_init * 180 / M_PI);

        setGPSDatum(msg->orientation);

        // we no longer need the orientation
        imuorient_sub.shutdown();
    } else if (imuDataCount <= IMU_SAMPLES) {
        yaw_mag_init += (float) yaw_mag;
        imuDataCount++;
    }
}

void PoseManager::gpsDataCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    gps_last = msg;
    if (datum_set) this->gps_pub.publish(msg);
}

void PoseManager::heightCallback(const std_msgs::Float64::ConstPtr &msg) {
    height_last = (float) msg->data;
}

void PoseManager::publishData(float px, float py) {
    geometry_msgs::PoseStamped posest;
    geometry_msgs::Pose pose;
    geometry_msgs::Point pt;

    pt.x = px;
    pt.y = py;
    pt.z = height_last;

    pose.position = pt;
    posest.pose = pose;
    posest.header.stamp = ros::Time::now();
    posest.header.frame_id = "map";
    this->point_pub.publish(posest);
}

void PoseManager::setGPSDatum(geometry_msgs::Quaternion orientation_last) {
    geographic_msgs::GeoPoint datumPosition;
    datumPosition.latitude = gps_last->latitude;
    datumPosition.longitude = gps_last->longitude;
    datumPosition.altitude = gps_last->altitude;

    geographic_msgs::GeoPose datumPose;
    datumPose.position = datumPosition;
    datumPose.orientation = orientation_last;

    orb_slam3::SetDatum setDatum;
    setDatum.request.geo_pose = datumPose;

    if (set_datum_client.call(setDatum)) {
        ROS_WARN("GLOBAL GPS HOME LOCATION (DATUM) set and propagated to other services");
        ROS_WARN("DATUM: lat: %f, lon: %f, alt: %f", datumPosition.latitude, datumPosition.longitude,
                 datumPosition.altitude);

        datum_set = true;

        publishData(0, 0);
    } else {
        ROS_ERROR("FAILED TO PROPAGATE GLOBAL GPS HOME LOCATION (DATUM)");
    }
}
