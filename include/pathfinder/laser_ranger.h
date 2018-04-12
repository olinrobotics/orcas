#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/LaserScan.h"
#include "distance_estimator.h"

using namespace cv;

class LaserRanger {
public:
    LaserRanger(std::string calibration_file);

    bool Step(const Mat& frame);

    Mat FindLaserMask(const Mat& frame);
    std::unique_ptr<sensor_msgs::LaserScan> FindLaserCOMs(const Mat& mask);

private:
    DistanceEstimator distance_estimator_;
    ros::NodeHandle nh_;
    ros::Publisher pub_;
};
