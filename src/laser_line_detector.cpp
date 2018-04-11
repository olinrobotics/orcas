#include "laser_line_detector.h"

using namespace cv;

// hsv
const Scalar GREEN_LOW(0, 0, 0);
const Scalar GREEN_HIGH(255, 140, 255);

bool CHOP_OFF_TOP = true;  // usually just reflections, never valid
bool CHOP_OFF_SIDES = true;  // usually garbage data

const float CAMERA_H_FOV = 64.4f;  // degrees
const unsigned int CAMERA_H_PX = 1280;  // px

LaserLineDetector::LaserLineDetector() : distance_estimator_(90.8170192922184f, -15.60448606032729f,
                                                             432.79093297611286f, 0.09326753247757161f),
                                         nh_(),
                                         pub_(nh_.advertise<sensor_msgs::LaserScan>("structured_light_scan", 10)) {
}

bool LaserLineDetector::Step(const Mat &frame) {
    Mat laser_mask = FindLaserMask(frame);
    // populate initial laser scan object
    std::unique_ptr<sensor_msgs::LaserScan> laser_scan = FindLaserCOMs(laser_mask);

    // fill out the scan ranges
    distance_estimator_.CalculateDistances(*laser_scan);

    // publish filled out scan message
    pub_.publish(*laser_scan);

    return true;
}

Mat LaserLineDetector::FindLaserMask(const Mat &frame) {
    Mat hsv;
    Mat mask;

    cvtColor(frame, hsv, CV_BGR2HSV);
    inRange(hsv, GREEN_LOW, GREEN_HIGH, mask);

    int w = frame.cols;
    int h = frame.rows;

    if (CHOP_OFF_TOP) {
        rectangle(mask, Point(0, 0), Point(w, h / 4), Scalar(0, 0, 0), -1);
    }
    if (CHOP_OFF_SIDES) {
        rectangle(mask, Point(0, 0), Point(w / 5, h), Scalar(0, 0, 0), -1);
        rectangle(mask, Point(w - (w / 5), 0), Point(w, h), Scalar(0, 0, 0), -1);
    }

    return mask;
}

std::unique_ptr<sensor_msgs::LaserScan> LaserLineDetector::FindLaserCOMs(const Mat &mask) {
    // TODO(danny): actually get the time of the image
    sensor_msgs::LaserScan scan;
    ros::Time scan_time = ros::Time::now();
    scan.header.stamp = scan_time;
    scan.header.frame_id = "webcam_frame";
    scan.angle_min = -CAMERA_H_FOV / 2.0f;
    scan.angle_max = CAMERA_H_FOV / 2.0f;
    scan.angle_increment = CAMERA_H_FOV / (float) CAMERA_H_PX;
    scan.range_min = 0.0f;
    scan.range_max = 1000.0f;

    assert(CAMERA_H_PX == mask.cols);  // we are making this assumption

    // bastardization of this field, these are center of masses (pixel heights)
    scan.intensities.resize(CAMERA_H_PX);
    scan.ranges.resize(CAMERA_H_PX);

    // it would be really awesome if I wanted to get rows instead of cols
    unsigned char *input = mask.data;
    for (int i = 0; i < mask.cols; i++) {
        int total_mass = 0;  // mass
        int total_mass_weighted = 0;  // mass * position
        for (int j = 0; j < mask.rows; j++) {
            unsigned char g = input[(int) mask.step * j + i];  // gray
            total_mass_weighted += j * g;
            total_mass += g;
        }
        if (total_mass > 0) {
            // calculate center of mass position
            scan.intensities[i] = (float) total_mass_weighted / total_mass;
        } else {
            scan.intensities[i] = 0.0f;
        }
    }

    return std::make_unique<sensor_msgs::LaserScan>(scan);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "laser_detector");
    ros::NodeHandle nh;
    LaserLineDetector ll;
    Mat frame;

    std::string filename = "/opt/catkin_ws/src/robosys-cv/test_data/water.mp4";
    VideoCapture capture(filename);

    if (!capture.isOpened()) {
        std::cerr << "Can't find file\n";
        char buff[FILENAME_MAX];
        getcwd(buff, FILENAME_MAX);
        std::cerr << buff;
        return 1;
    }

    while (true) {
        capture >> frame;
        if (frame.empty()) {
            break;
        }
        ll.Step(frame);
    }

    return 0;
}
