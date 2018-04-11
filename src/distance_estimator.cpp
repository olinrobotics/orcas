#include <fstream>
#include "distance_estimator.h"

DistanceEstimator::DistanceEstimator(float a, float b, float c, float theta) :
        a_(a), b_(b), c_(c), theta_(theta), slope_(tanf(theta)) {
}

float GetNextNumber(std::stringstream& line_stream, bool comma_separated=true) {
    std::string num_string;
    if (comma_separated) {
        std::getline(line_stream, num_string, ',');
    } else {
        std::getline(line_stream, num_string);
    }

    return static_cast<float>(std::atof(num_string.c_str()));
}

DistanceEstimator::DistanceEstimator(std::string calibration_path) {
    std::ifstream calibration_file(calibration_path);
    // ugly way of reading the super simple configuration file
    std::string line;
    std::getline(calibration_file, line);  // skip a line

    std::getline(calibration_file, line);  // read a,b,c
    std::stringstream abc_stream(line);
    a_ = GetNextNumber(abc_stream);
    b_ = GetNextNumber(abc_stream);
    c_ = GetNextNumber(abc_stream);

    std::cerr << "Loading calibration from file: " << calibration_path << "\n";
    std::cerr << "a: " << a_ << "\n";
    std::cerr << "b: " << b_ << "\n";
    std::cerr << "c: " << c_ << "\n";

    std::getline(calibration_file, line);  // skip a line
    std::getline(calibration_file, line);  // read theta
    std::stringstream theta_stream(line);
    theta_ = GetNextNumber(theta_stream, false);
    std::cerr << "theta: " << theta_ << "\n";
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
