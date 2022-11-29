#! /usr/bin/env python3

from types import GeneratorType
import rospy
import cv2 as cv
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from crazyflie_messages.msg import OpticalFlow


class OpticFlowCalculator():
    def __init__(self):
        # Initialize parameters
        self.prev_gray = None
        self.gray = None
        self.bgr8 = None
        self.prev = None
        self.next = None
        self.status = None
        self.error = None
        self.good_old = None
        self.good_new = None
        self.mask = None
        self.i = None
        self.new = None
        self.old = None
        self.a = None
        self.b = None
        self.c = None
        self.d = None
        self.image = None
        self.output =None

        #Node cycle rate
        self.rate = rospy.Rate(10)

        # Parameters for ShiTomasi corner detection
        self.feature_params = dict( maxCorners = 100, qualityLevel = 0.3, minDistance = 7, blockSize = 7)

        # Parameters for Lucas-Kanade optical flow
        self.lk_params = dict(winSize = (15,15), maxLevel = 2, criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))

        # Color to draw optical flow track
        self.color = (0,255,0)

        # Publisher and subscribers set up
        self.image_sub = rospy.Subscriber('/cf1/camera1/image_raw', Image, self.camera_callback)
        self.optic_flow_pub = rospy.Publisher("/cf1/optic_flow", OpticalFlow, queue_size=10)
        self.optic_flow_pub_mean = rospy.Publisher("/cf1/optic_flow_mean_vx", Float32, queue_size=10)
        
        self.bridge = CvBridge()


    def camera_callback(self,image):
        try:
            self.bgr8 = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            self.gray = cv.cvtColor(self.bgr8, cv.COLOR_BGR2GRAY)
        except CvBridgeError as e:
            print(e)

        # If this is the first loop    
        if self.prev_gray is None:
            self.prev_gray = self.gray
            # Find the strongest corners in the first frame by Shi-Tomasi method, we will track the optical flow for these corners
            self.prev = cv.goodFeaturesToTrack(self.prev_gray, mask=None, **self.feature_params)
            # Creates an image filled with zero intensities with the same dimensions as the first frame---for later drawing purposes
            self.mask = np.zeros_like(self.bgr8)
            return

         

        # Calculates sparse optical flow by Lucas-Kanade method
        self.next, self.status, self.error = cv.calcOpticalFlowPyrLK(self.prev_gray, self.gray, self.prev.astype(np.float32), None, **self.lk_params)

        # Select good feature points for previous position
        self.good_old = self.prev[self.status == 1].astype(int)

        # Select good feature points for next position
        self.good_new = self.next[self.status == 1].astype(int)

        # Draw the optical flow tracks
        for self.i, (self.new, self.old) in enumerate(zip(self.good_new, self.good_old)):
            # Return a contiguous flattened array as (x,y) coordinates for new point
            self.a, self.b = self.new.ravel()

            # Return a contiguous flattened array as (x,y) coordinates for old point
            self.c, self.d = self.old.ravel()

            # Draw lines between new and old position with green color and 2 thickness
            # self.mask = cv.line(self.mask, (self.a, self.b), (self.c, self.d), self.color, 2)

            # Draw filled circle (thickness of -1) at new position with green color and radius of 3
            # self.image = cv.circle(self.bgr8, (self.a, self.b), 1, self.color, -1)

        self.output = cv.line(self.bgr8, (self.a, self.b), (self.c, self.d), self.color, 2)
        # Overlays the optical flow tracks on the original frame
        # self.output = cv.add(self.image, self.mask)

        # Update previous frame
        self.prev_gray = self.gray

        # # Update previous good feature points
        # self.prev = self.good_new.reshape(-1,1,2)

        # Show the images:grayscale, bgr8, optical flow    
        cv.imshow("grayscale", self.gray)
        cv.imshow("bgr8", self.bgr8)
        cv.imshow("sparse optical flow", self.output)

        # Update the frame every 10 ms
        cv.waitKey(10)

def main():
    rospy.init_node("sparse_solution")
    my_node = OpticFlowCalculator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')
        cv.destroyAllWindows()

if __name__ == '__main__':
    main()