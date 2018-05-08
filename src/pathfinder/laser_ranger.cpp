#include <ros/package.h>
#include "pathfinder/laser_ranger.h"
#include "pathfinder/file_finder.h"

// NOTE(danny): if the masked laser image isn't right, use to debug
//#define SHOW_IMAGE

using namespace cv;

// rgb
const Scalar kMaskLow(0);
const Scalar kMaskHigh(255);

// gray
const Scalar kBrightLow(130);
const Scalar kBrightHigh(255);

bool kChopOffTop = true;  // usually just reflections, never valid
bool kChopOffSides = true;  // usually garbage data

// TOmaybeDO(danny): store in calibration file
const float CAMERA_H_FOV = 64.4f / 180.0f * 3.14159f;  // degrees -> radians
const unsigned int CAMERA_H_PX = 1280;  // px

LaserRanger::LaserRanger(std::string calibration_file) : distance_estimator_(calibration_file),
                                         nh_(),
                                         pub_(nh_.advertise<sensor_msgs::LaserScan>("structured_light_scan", 10)) {
}

bool LaserRanger::Step(const Mat &gray_frame) {
    Mat frame_mask = FindFrameMask(gray_frame);
    Mat highlight_mask;
    Mat highlight_values;
    Mat gray_output;

    // original frame -> highlight bright parts into a mask
    // highlight likely laser parts
    inRange(gray_frame, kBrightLow, kBrightHigh, highlight_mask);

    // cut out laser values with highlight mask
    bitwise_and(gray_frame, gray_frame, highlight_values, highlight_mask);

    addWeighted(gray_frame, 0.5f, highlight_values, 60.0f / 255.0f, 0.0, gray_output);

    // ignore part of the image
    bitwise_and(gray_output, gray_output, gray_output, frame_mask);
    #ifdef SHOW_IMAGE
    imshow("window", gray_output);
    waitKey(10);
    #endif
    // populate initial laser scan object
    std::unique_ptr<sensor_msgs::LaserScan> laser_scan = FindLaserCOMs(gray_output);

    // fill out the scan ranges
    distance_estimator_.CalculateDistances(*laser_scan);

    // publish filled out scan message
    pub_.publish(*laser_scan);

    return true;
}

Mat LaserRanger::FindFrameMask(const Mat &gray_frame) {
    Mat mask;

    inRange(gray_frame, kMaskLow, kMaskHigh, mask);

    int w = gray_frame.cols;
    int h = gray_frame.rows;

    if (kChopOffTop) {
        rectangle(mask, Point(0, 0), Point(w, h / 4), Scalar(0), -1);
    }
    if (kChopOffSides) {
        rectangle(mask, Point(0, 0), Point(w / 5, h), Scalar(0), -1);
        rectangle(mask, Point(w - (w / 5), 0), Point(w, h), Scalar(0), -1);
    }

    return mask;
}

std::unique_ptr<sensor_msgs::LaserScan> LaserRanger::FindLaserCOMs(const Mat &gray) {
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

    assert(CAMERA_H_PX == gray.cols);  // we are making this assumption

    // bastardization of this field, these are center of masses (pixel heights)
    scan.intensities.resize(CAMERA_H_PX);
    scan.ranges.resize(CAMERA_H_PX);

    Mat gray_float;
    gray.convertTo(gray_float, CV_32FC1);

    // it would be really awesome if I wanted to get rows instead of cols
    for (int col = 0; col < gray_float.cols; col++) {
        float total_unaltered_mass = 0;
        float total_mass = 0;  // mass
        float total_mass_weighted = 0;  // mass * position
        for (int row = 0; row < gray_float.rows; row++) {
            float g = gray_float.at<float>(row, col) / 255.0f;
            total_unaltered_mass += g;
            g = powf(g, 6.0f);
            //unsigned char g = input[(int) mask.step * j + i];  // gray
            total_mass_weighted += row * g;
            total_mass += g;
        }

        if (total_unaltered_mass > static_cast<float>(gray_float.rows) / 36.0f) {
            // calculate center of mass position
            scan.intensities[col] = (total_mass_weighted / total_mass) / static_cast<float>(gray_float.rows);
        } else {
            scan.intensities[col] = 0.0f;
        }
    }

    return std::make_unique<sensor_msgs::LaserScan>(scan);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "laser_detector");
    ros::NodeHandle nh;
    Mat frame;
    VideoCapture capture;

    #ifdef SHOW_IMAGE
    namedWindow("window", WINDOW_AUTOSIZE);
    #endif
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

    std::cerr << "Initialized. Starting loop soon...\n";
    while (ros::ok()) {
        capture >> frame;
        if (frame.empty()) {
            break;
        }
        Mat gray_frame;
        // extract original green channel into a gray image
        cv::extractChannel(frame, gray_frame, 1);
        // flip frame horizontally
        flip(gray_frame, gray_frame, +1);
        ranger.Step(gray_frame);
    }

    return 0;
}
