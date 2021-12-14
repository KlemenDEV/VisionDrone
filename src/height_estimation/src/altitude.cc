/*
    altitude.cpp: Altitude estimation via barometer/accelerometer fusion
*/

#include "filters.h"
#include "algebra.h"
#include "altitude.h"

AltitudeEstimator::AltitudeEstimator(float sigmaAccel, float sigmaBaro, float accelThreshold) : complementary(
        sigmaAccel, sigmaBaro, accelThreshold) {
    this->sigmaAccel = sigmaAccel;
    this->sigmaBaro = sigmaBaro;
    this->accelThreshold = accelThreshold;
}

void AltitudeEstimator::estimate(geometry_msgs::Quaternion orient, float accel[3], float baroHeight, float dt) {
    tf2::Quaternion tf_orient;
    tf2::fromMsg(orient, tf_orient);
    tf_orient.normalize();
    tf2::Matrix3x3 R(tf_orient);
    tf2::Vector3 tf_accel(accel[0], accel[1], accel[2]);

    tf2::Vector3 tf_accel_earth = tf_accel * R;

    float verticalAccel = tf_accel_earth[2] - 1;

    complementary.estimate(&estimatedVelocity,
                           &estimatedAltitude,
                           baroHeight,
                           pastAltitude,
                           pastVerticalVelocity,
                           pastVerticalAccel,
                           dt);

    pastAltitude = estimatedAltitude;
    pastVerticalVelocity = estimatedVelocity;
    pastVerticalAccel = verticalAccel;
}

float AltitudeEstimator::getAltitude() {
    // return the last estimated altitude
    return estimatedAltitude;
}

float AltitudeEstimator::getVerticalVelocity() {
    // return the last estimated vertical velocity
    return estimatedVelocity;
}

float AltitudeEstimator::getVerticalAcceleration() {
    // return the last estimated vertical acceleration
    return pastVerticalAccel;
}