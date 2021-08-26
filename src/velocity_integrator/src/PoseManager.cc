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
}

void PoseManager::imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    tf2::Quaternion quat_gps_rot;
    tf2::fromMsg(orientation_last, quat_gps_rot);
    tf2::Matrix3x3 gps_tf2_rot(quat_gps_rot);
    double r, p, yaw_mag;
    gps_tf2_rot.getRPY(r, p, yaw_mag, 1);

    yaw_mag_curr = (float) yaw_mag;
    orientation_last = msg->orientation;
}

void PoseManager::gpsDataCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    gps_last = msg;
    if (datum_set) this->gps_pub.publish(msg);
}

void PoseManager::heightCallback(const std_msgs::Float64::ConstPtr &msg) {
    height_last = (float) msg->data;
}

void PoseManager::publishData(float px, float py) {
    double dt = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - time_last).count() / 1000.0;

    float vx = (px - opx) / dt;
    float vy = (py - opy) / dt;

    opx = px;
    opy = py;
    time_last = std::chrono::high_resolution_clock::now();

    if (!datum_set) {
        if (flightState == REST) {
            if (height_last > 7) flightState = TAKEOFF;
            ROS_WARN_THROTTLE(1.0, "State: REST");
            return;
        } else if (flightState == TAKEOFF) {
            if (vx > 7) flightState = FRONT_FLIGHT;
            ROS_WARN_THROTTLE(1.0, "State: TAKEOFF");
            return;
        } else if (flightState == FRONT_FLIGHT) {
            if (vx < 7 || abs(vy) > 4) return;

            double yaw_est = atan2(vy, vx);

            yaw_diff += yaw_mag_curr - yaw_est;
            yaw_mag_avg += yaw_mag_curr;
            yaw_diff_count++;

            if (yaw_diff_count > 10) {
                yaw_diff /= (double) yaw_diff_count;
                yaw_mag_avg /= (double) yaw_diff_count;

                setGPSDatum(px, py);
                flightState = FREE_FLIGHT;
            } else {
                ROS_WARN_THROTTLE(1.0, "State: FRONT_FLIGHT");
                return;
            }
        }
    }

    geometry_msgs::PoseStamped posest;
    geometry_msgs::Pose pose;
    geometry_msgs::Point pt;

    pt.x = (px - offx) * cos(yaw_diff);
    pt.y = (py - offy) * sin(yaw_diff);
    pt.z = height_last;

    pose.position = pt;
    posest.pose = pose;
    posest.header.stamp = ros::Time::now();
    posest.header.frame_id = "map";
    this->point_pub.publish(posest);
}

void PoseManager::setGPSDatum(float px, float py) {
    if (datum_set) return;

    yaw_mag_init = yaw_mag_avg;
    ROS_WARN("Initial yaw: %f deg", yaw_mag_avg * 180 / M_PI);

    double yaw_offset_mag = yaw_mag_avg - yaw_mag_curr;
    yaw_diff += yaw_offset_mag;

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

        offx = px;
        offy = py;
        datum_set = true;
    } else {
        ROS_ERROR("FAILED TO PROPAGATE GLOBAL GPS HOME LOCATION (DATUM)");
    }
}
