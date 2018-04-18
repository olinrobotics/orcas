#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer
#include "pathfinder/file_finder.h"

int main(int argc, char** argv)
{
  // Check if video source has been passed as a parameter
  if (argv[1] == NULL) return 1;

  ros::init(argc, argv, "pathfinder_cam_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  std::string pkg_path = ros::package::getPath("orcas");

  VideoCapture cap;
  std::string first_arg(argv[1]);
  LoadCaptureArg(cap, pkg_path, first_arg);

  // double-check if video device is opened
  if(!cap.isOpened()) return 1;
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(15);
  while (nh.ok()) {
    cap >> frame;
    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      ROS_DEBUG("Publishing image");
      pub.publish(msg);
      cv::waitKey(1);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
