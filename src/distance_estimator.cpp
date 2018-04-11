#include "distance_estimator.h"

DistanceEstimator::DistanceEstimator(float a, float b, float c, float theta) :
        a_(a), b_(b), c_(c), theta_(theta), slope_(tanf(theta)) {
}

void DistanceEstimator::CalculateDistances(
        sensor_msgs::LaserScan &laser_scan) {
    // use a laser scan, populate the ranges field

    unsigned long width = laser_scan.intensities.size();

    for (unsigned int i = 0; i < width; i++) {
        // center x at the middle of the image
        float x = (float)i - ((float)width / 2.0f);

        // if we didn't get a measurement, just set range to nan and go to the next
        if (laser_scan.intensities[i] == 0.0f) {
            laser_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
            continue;
        }

        // correct the height of the pixel based on the angle of the laser
        float corrected_height = laser_scan.intensities[i] - x * slope_;

        // calculate range in cm based on calibrated equation coefs
        laser_scan.ranges[i] = a_ + b_ * logf(corrected_height - c_);
    }
}
