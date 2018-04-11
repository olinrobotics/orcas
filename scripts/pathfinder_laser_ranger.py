#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('orcas')
import sys
import rospy
import cv2
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError

from laser_distance_estimator import LaserLineDetector

CAMERA_H_FOV = 64.4  # degrees
CAMERA_H_PX = 1280  # px


class LaserRanger(object):

    def __init__(self, camera_id):
        rospy.init_node('laser_ranger')

        self.scan_pub = rospy.Publisher("structured_light_scan", LaserScan, queue_size=10)
        self.cap = None
        self.camera_id = camera_id

        if self.camera_id is None:
            self.bridge = CvBridge()
            self.image_sub = rospy.Subscriber("camera/image", Image, self.on_image)
        else:
            self.cap = cv2.VideoCapture(self.camera_id)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,10000)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,10000)

        self.ll = LaserLineDetector(camera=None)

    def on_image(self, data):
        if self.camera_id is None:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)
                return
        else:
            ret, cv_image = self.cap.read()

        (rows, cols, channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (50,50), 10, 255)

        self.ll.step(cv_image)
        self.ll.find_distances()

        height, width, channels = self.ll.cur_frame.shape

        self.scan_pub.publish(
            header=Header(
                stamp=rospy.Time.now(),  # TODO: use camera image time
                frame_id='cam'
            ),
            angle_min=-CAMERA_H_FOV / 2.,
            angle_max=CAMERA_H_FOV / 2.,
            angle_increment=CAMERA_H_FOV / width,
            time_increment=0.0,
            scan_time=1.0/30.0,
            range_min=0.0,
            range_max=50.0,
            ranges=self.ll.cur_distances,
        )

def main(args):
    if len(args) == 1:
        try:
            camera_id = int(sys.argv[1])
        except ValueError:
            camera_id = sys.argv[1]
    else:
        camera_id = None

    ic = LaserRanger(camera_id=camera_id)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        if ic.cap is not None:
            ic.cap.release()

if __name__ == '__main__':
    main(sys.argv)
