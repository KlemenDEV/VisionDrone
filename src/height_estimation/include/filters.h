/*
   filters.h: Filter class declarations
 */

#pragma once

#include <math.h>
#include <stdint.h>

#include "algebra.h"

class ComplementaryFilter {

private:

    // filter gain
    float gain[2];
    // Zero-velocity update
    float accelThreshold;
    static const uint8_t ZUPT_SIZE = 12;
    uint8_t ZUPTIdx;
    float ZUPT[ZUPT_SIZE];

    float ApplyZUPT(float accel, float vel);

public:

    ComplementaryFilter(float sigmaAccel, float sigmaBaro, float accelThreshold);

    void estimate(float *velocity, float *altitude, float baroAltitude,
                  float pastAltitude, float pastVelocity, float accel, float deltat);
}; // Class ComplementaryFilter