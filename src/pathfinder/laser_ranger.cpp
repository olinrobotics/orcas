#include <ros/package.h>
#include "pathfinder/laser_ranger.h"
#include "pathfinder/file_finder.h"

using namespace cv;

#define INVERT_HORIZONTAL true

// hsv
const Scalar kGreenLow(0, 0, 0);
const Scalar kGreenHigh(255, 140, 255);

bool kChopOffTop = true;  // usually just reflections, never valid
bool kChopOffSides = true;  // usually garbage data

// TOmaybeDO(danny): store in calibration file
const float CAMERA_H_FOV = 64.4f / 180.0f * 3.14159f;  // degrees -> radians
const unsigned int CAMERA_H_PX = 1280;  // px

LaserRanger::LaserRanger(std::string calibration_file) : distance_estimator_(calibration_file),
                                         nh_(),
                                         pub_(nh_.advertise<sensor_msgs::LaserScan>("structured_light_scan", 10)) {
}

bool LaserRanger::Step(const Mat &frame) {
    Mat laser_mask = FindLaserMask(frame);
    // populate initial laser scan object
    std::unique_ptr<sensor_msgs::LaserScan> laser_scan = FindLaserCOMs(laser_mask);

    // fill out the scan ranges
    distance_estimator_.CalculateDistances(*laser_scan);

    // publish filled out scan message
    pub_.publish(*laser_scan);

    return true;
}

Mat LaserRanger::FindLaserMask(const Mat &frame) {
    Mat hsv;
    Mat mask;

    cvtColor(frame, hsv, CV_BGR2HSV);
    inRange(hsv, kGreenLow, kGreenHigh, mask);

    int w = frame.cols;
    int h = frame.rows;

    if (kChopOffTop) {
        rectangle(mask, Point(0, 0), Point(w, h / 4), Scalar(0, 0, 0), -1);
    }
    if (kChopOffSides) {
        rectangle(mask, Point(0, 0), Point(w / 5, h), Scalar(0, 0, 0), -1);
        rectangle(mask, Point(w - (w / 5), 0), Point(w, h), Scalar(0, 0, 0), -1);
    }

    return mask;
}

std::unique_ptr<sensor_msgs::LaserScan> LaserRanger::FindLaserCOMs(const Mat &mask) {
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
            #if INVERT_HORIZONTAL
            scan.intensities[mask.cols - i - 1] = (float) total_mass_weighted / total_mass;
            #else
            scan.intensities[i] = (float) total_mass_weighted / total_mass;
            #endif
        } else {
            #if INVERT_HORIZONTAL
            scan.intensities[mask.cols - i - 1] = 0.0f;
            #else
            scan.intensities[i] = 0.0f;
            #endif
        }
    }

    return std::make_unique<sensor_msgs::LaserScan>(scan);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "laser_detector");
    ros::NodeHandle nh;
    Mat frame;
    VideoCapture capture;

    // find the package root (orcas)
    std::string pkg_path = ros::package::getPath("orcas");

    // require an argument to launch, the camera/path to a video
    if (argv[1] == NULL) {
        std::cerr << "Need to launch with a camera argument!" <<
            "(e.g. 0 or a/path/from/orcas/video.mp4)\n";
        return 1;
    }
    std::string first_arg(argv[1]);

    LoadCaptureArg(capture, pkg_path, first_arg);

    // load calibration file to make ranger
    std::string calibration_path = pkg_path + "/data/calibration.txt";
    LaserRanger ranger(calibration_path);

    std::cerr << "Initialized. Starting loop...\n";
    while (true) {
        capture >> frame;
        if (frame.empty()) {
            break;
        }
        ranger.Step(frame);
    }

    return 0;
}
