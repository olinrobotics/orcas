#!/usr/bin/env python
'''
Code to make the pool camera work with the submarine for ORCAS.
Spring 2017
Adapted by Vicky McDermott from:
https://github.com/olinrobotics/edwin/blob/master/scripts/Interactions/PlayPushCup.py
'''
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import numpy as np

class PoolCam:
    def __init__(self):
        '''
        Initializes the PoolCam object to be used
        to gather raw camera data that can be sent to
        the submarine.

        '''
        self.cv_image = None
        # Creates node from which to subcribe and publish data
        rospy.init_node('pool_cam')
        # Gets data from usb_cam
        self.bridge = CvBridge()
        # Determines node for subscription
        self.image_sub = rospy.Subscriber("/camera/image_raw",Image,self.callback)

    # Runs once for every reciept of an image from usb_cam
    def callback(self, data):
        '''
        Runs once for every reciept of an image from the camera.
        Gets the image and converts it to an opencv image.
        '''

        try:
            # Converts usb cam feed to csv feed; bgr8 is an image encoding
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Sets image size
            image_size = self.cv_image.shape
            screen_height = image_size[0]
            screen_width = image_size[1]

        except CvBridgeError as e:
            print(e)

    def capture_images(self, image):
        '''
        Captures and displays raw camera feed and also
        displays the feed.
        '''

        if image is None:
            return

        # Feed Display(s):
        cv2.imshow('Raw Feed (feed)',image)
        # would like to understand why this makes video stream
        k = cv2.waitKey(5) & 0xFF

    def apply_filter(self, image):
        '''
        '''
        if image is None:
            return

        blur = cv2.GaussianBlur(image, (5,5), 0) # Gaussian Blur filter
        #cv2.imshow('Gaussian Blur', blur)
        #k = cv2.waitKey(5) & 0xFF
        # Changes BGR video to GRAY and threshold it
        vidgray = cv2.cvtColor(blur,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(vidgray,200,255,cv2.THRESH_BINARY_INV)
        return thresh

    def contour_feed(self, feed, thresh):
        '''
        '''
        if self.cv_image is None:
            return

        _, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        sub_contours = [contour for contour in contours if cv2.contourArea(contour) > 300 and
                        cv2.contourArea(contour) < 500000]
        circle_contours = [contour for contour in contours if cv2.contourArea(contour) > 300 and
                        cv2.contourArea(contour) < 500000 and
                        len(cv2.approxPolyDP(contour,0.01*cv2.arcLength(contour,True),True)) > 15]
        '''for contour in sub_contours:
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(feed,[box],0,(0,0,255),2)'''
        '''rect = cv2.minAreaRect(sub_contours[0])
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(feed,[box],0,(0,0,255),2)'''

        cv2.drawContours(feed, sub_contours, -1, (0,255,0), 3)
        # cv2.drawContours(feed, circle_contours, -1, (255,0,0), 3)



    def run(self):
        r = rospy.Rate(20) # Sets update rate
        while not rospy.is_shutdown():
            r.sleep()
            # if self.debug == True: self.calibrate()
            self.capture_images(self.cv_image)
            vid = self.apply_filter(self.cv_image)
            self.contour_feed(self.cv_image, vid)

if __name__=='__main__':
    '''
    Test the camera
    '''
    pc = PoolCam() # Creates new instance of class PoolCam
    pc.run() # Calls run function
