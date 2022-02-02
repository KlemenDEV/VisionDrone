/*
    altitude.h: Altitude estimation via barometer/accelerometer fusion
*/

# pragma once

#include "filters.h"
#include "algebra.h"

#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class AltitudeEstimator {

private:
    // required parameters for the filters used for the estimations
    // sensor's standard deviations
    float sigmaAccel;
    float sigmaBaro;

    // Zero-velocity update acceleration threshold
    float accelThreshold;

    // For computing the sampling period
    // required filters for altitude and vertical velocity estimation
    ComplementaryFilter complementary;
    // Estimated past vertical acceleration
    float pastVerticalAccel = 0;
    float pastVerticalVelocity = 0;
    float pastAltitude = 0;

    // estimated altitude and vertical velocity
    float estimatedAltitude = 0;
    float estimatedVelocity = 0;

public:
    AltitudeEstimator(float sigmaAccel, float sigmaBaro, float accelThreshold);

    void estimate(geometry_msgs::Quaternion orient, float accel[3], float baroHeight, float dt);

    float getAltitude();

    float getVerticalVelocity();

    float getVerticalAcceleration();

}; // class AltitudeEstimator