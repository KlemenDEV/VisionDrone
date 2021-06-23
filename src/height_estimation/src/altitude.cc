/*
    altitude.cpp: Altitude estimation via barometer/accelerometer fusion
*/

#include "filters.h"
#include "algebra.h"
#include "altitude.h"

AltitudeEstimator::AltitudeEstimator(float sigmaAccel, float sigmaGyro, float sigmaBaro,
                                     float ca, float accelThreshold)
        : kalman(ca, sigmaGyro, sigmaAccel), complementary(sigmaAccel, sigmaBaro, accelThreshold) {
    this->sigmaAccel = sigmaAccel;
    this->sigmaGyro = sigmaGyro;
    this->sigmaBaro = sigmaBaro;
    this->ca = ca;
    this->accelThreshold = accelThreshold;
}

void AltitudeEstimator::estimate(float accel[3], float gyro[3], float baroHeight, float dt) {
    float verticalAccel = kalman.estimate(pastGyro,
                                          pastAccel,
                                          dt);
    complementary.estimate(&estimatedVelocity,
                           &estimatedAltitude,
                           baroHeight,
                           pastAltitude,
                           pastVerticalVelocity,
                           pastVerticalAccel,
                           dt);
    // update values for next iteration
    copyVector(pastGyro, gyro);
    copyVector(pastAccel, accel);
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