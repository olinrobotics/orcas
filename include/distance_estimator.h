#include "sensor_msgs/LaserScan.h"

class DistanceEstimator {
public:
    DistanceEstimator(float a, float b, float c, float theta);
    void CalculateDistances(sensor_msgs::LaserScan& center_of_masses);
private:
    const float a_, b_, c_, theta_, slope_;
};
