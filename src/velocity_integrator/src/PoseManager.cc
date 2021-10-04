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

    set_datum_client = nh->serviceClient<velocity_integrator::SetDatum>("/datum");
    set_datum_client_ref = nh->serviceClient<velocity_integrator::SetDatum>("/datum_ref");
    set_datum_client_sim = nh->serviceClient<velocity_integrator::SetDatum>("/datum_sim");
}

#define ALPHA 0.85

void PoseManager::imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    orientation_last = msg->orientation;

    tf2::Quaternion quat_gps_rot;
    tf2::fromMsg(orientation_last, quat_gps_rot);
    tf2::Matrix3x3 gps_tf2_rot(quat_gps_rot);
    double r, p, yaw;
    gps_tf2_rot.getRPY(r, p, yaw, 1);

    yaw_mag_curr = (yaw * ALPHA) + (1.0 - ALPHA) * yaw_mag_curr;

    if (datum_set) yaw_last = yaw_mag_init - yaw_mag_curr;
}

void PoseManager::gpsDataCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    gps_last = msg;
    if (datum_set) this->gps_pub.publish(msg);
}

void PoseManager::heightCallback(const std_msgs::Float64::ConstPtr &msg) {
    height_last = msg->data;
}

void PoseManager::publishData(double px, double py) {
    if (!datum_set) setGPSDatum();

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

void PoseManager::setGPSDatum() {
    datum_set = true;

    geographic_msgs::GeoPose datumPose;
    geographic_msgs::GeoPoint datumPosition;
    datumPosition.latitude = gps_last->latitude;
    datumPosition.longitude = gps_last->longitude;
    datumPosition.altitude = gps_last->altitude - height_last;
    datumPose.position = datumPosition;
    datumPose.orientation = orientation_last;

    velocity_integrator::SetDatum setDatum;
    setDatum.request.geo_pose = datumPose;

    if (set_datum_client.call(setDatum)) {
        ROS_WARN("GLOBAL GPS HOME LOCATION (DATUM) set and propagated to other services");
        ROS_WARN("DATUM: lat: %f, lon: %f, alt: %f", datumPosition.latitude, datumPosition.longitude,
                 datumPosition.altitude);

        yaw_mag_init = yaw_mag_curr;
        set_datum_client_ref.call(setDatum);
        set_datum_client_sim.call(setDatum);
    } else {
        ROS_ERROR("FAILED TO PROPAGATE GLOBAL GPS HOME LOCATION (DATUM)");
    }
}
