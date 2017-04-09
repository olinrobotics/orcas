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

class PoolCam:
    def __init__():
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

    def capture_images(self):
        '''
        Captures and displays raw camera feed and also
        displays the feed with a gaussian blur filter applied.
        '''

        if self.cv_image == None:
            return

        # Gaussian Blur filter
        blur = cv2.GaussianBlur(self.cv_image, (5,5), 0) # Gaussian Blur filter

        # Feed Display(s):
        cv2.imshow('Raw Feed (feed)',self.cv_image)
        cv2.imshow('Gaussian Blur Filter (blur)', self.cv_image)

if __name__=='__main__':
    '''
    Test the camera
    '''
    pc = PoolCam() # Creates new instance of class PoolCam
    pc.capture_images() # Calls run function
