#pragma once
#include <ros/ros.h>
#include <ros/package.h>

using namespace cv;

struct InvalidCaptureArg : public std::exception {
    const char * what() const throw () {
        return "Invalid Capture Argument";
    }
};

void LoadCaptureArg(VideoCapture& capture, std::string relative_path_root, std::string first_arg) {
    // check if arg qualifies as an integer
    std::istringstream first_arg_stream(first_arg);
    int camera_id;
    if (first_arg_stream >> camera_id) {
        // load live camera (/dev/video<camera_id>)
        capture.open(camera_id);
        if (!capture.isOpened()) {
            std::cerr << "Couldn't open camera " << camera_id
                << "\nDoes /dev/video" << camera_id << " exist?\n";
            throw InvalidCaptureArg();
        }

        // set to maximum resolution possible (was getting 640x480)
        capture.set(CV_CAP_PROP_FRAME_WIDTH, 10000);
        capture.set(CV_CAP_PROP_FRAME_HEIGHT, 10000);
    } else {
        std::string path_to_open;
        // couldn't coerce arg to integer, assume it's a path
        if (first_arg.length() && first_arg.at(0) == '/') {
            // absolute path
            path_to_open = first_arg;
        } else {
            // relative path, start at the repo root
            path_to_open = relative_path_root + "/" + first_arg;
        }

        capture.open(path_to_open);
        if (!capture.isOpened()) {
            std::cerr << "Can't open file\n";
            std::cerr << path_to_open + "\n";
            char buff[FILENAME_MAX];
            getcwd(buff, FILENAME_MAX);
            std::cerr << buff;
            throw InvalidCaptureArg();
        }
    }
}
