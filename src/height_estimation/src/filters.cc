/*
   filters.cpp: Filter class implementations
 */

//#include <cmath>
#include <stdlib.h> // XXX eventually use fabs() instead of abs() ?

#include "filters.h"


float ComplementaryFilter::ApplyZUPT(float accel, float vel) {
    // first update ZUPT array with latest estimation
    ZUPT[ZUPTIdx] = accel;
    // and move index to next slot
    uint8_t nextIndex = (ZUPTIdx + 1) % ZUPT_SIZE;
    ZUPTIdx = nextIndex;
    // Apply Zero-velocity update
    for (uint8_t k = 0; k < ZUPT_SIZE; ++k) {
        if (abs(ZUPT[k]) > accelThreshold) return vel;
    }
    return 0.0;
}


ComplementaryFilter::ComplementaryFilter(float sigmaAccel, float sigmaBaro, float accelThreshold) {
    // Compute the filter gain
    gain[0] = sqrt(2 * sigmaAccel / sigmaBaro);
    gain[1] = sigmaAccel / sigmaBaro;
    // If acceleration is below the threshold the ZUPT counter
    // will be increased
    this->accelThreshold = accelThreshold;
    // initialize zero-velocity update
    ZUPTIdx = 0;
    for (uint8_t k = 0; k < ZUPT_SIZE; ++k) {
        ZUPT[k] = 0;
    }
}

void ComplementaryFilter::estimate(float *velocity, float *altitude, float baroAltitude,
                                   float pastAltitude, float pastVelocity, float accel, float deltat) {
    // Apply complementary filter
    *altitude =
            pastAltitude + deltat * (pastVelocity + (gain[0] + gain[1] * deltat / 2) * (baroAltitude - pastAltitude)) +
            accel * pow(deltat, 2) / 2;
    *velocity = pastVelocity + deltat * (gain[1] * (baroAltitude - pastAltitude) + accel);
    // Compute zero-velocity update
    *velocity = ApplyZUPT(accel, *velocity);
}