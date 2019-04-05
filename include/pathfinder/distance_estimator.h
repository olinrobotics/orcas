#pragma once
#include "sensor_msgs/LaserScan.h"

class DistanceEstimator {
public:
    DistanceEstimator(float a, float b, float c, float theta);
    DistanceEstimator(std::string calibration_path);
    void CalculateDistances(sensor_msgs::LaserScan& center_of_masses);
private:
    float a_, b_, c_, theta_, slope_;
};

struct InvalidCalibrationFile : public std::exception {
    const char * what() const throw () {
        return "Invalid Calibration File";
    }
};
