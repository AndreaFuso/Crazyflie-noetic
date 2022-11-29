#! /usr/bin/env python3

# ROS imports
import rospy

# Opencv imports
import cv2

# numpy imports-basic math and matrix manipulation
import numpy as np
import math

# imports for ROS image handling
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# message imports specific to this package
from crazyflie_messages.msg import OpticalFlow


def draw_optic_flow_field(gray_image, points, flow):
    # gray_image: opencv gray image, e.g. shape = (width, height)
    # points: points at which optic flow is tracked, e.g. shape = (npoints, 1, 2)
    # flow: optic flow field, should be same shape as points

    color_img = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
    color_red = [0,255,0] # bgr colorspace
    linewidth = 2
    for i, point in enumerate(points):
        # Becareful that here for the cv.line, the datatype is int
        x = point[0,0].astype(int)
        y = point[0,1].astype(int)
        vx = flow[i][0,0].astype(int)
        vy = flow[i][0,1].astype(int)
        cv2.line(color_img, (x,y), (x+vx, y+vy), color_red, linewidth) # draw a red line from the point with vector = [vx, vy] 
    cv2.imshow('optic_flow_field',color_img)
    cv2.waitKey(1)

class OpticFlowCalculator():
    def __init__(self):
        # Define the source of the images, e.g. rostopic name

        # Initialize image acquisition
        self.bridge = CvBridge()
        self.prev_image = None
        self.last_time = 0
        # self.delta_x = 0
        # self.delta_y = 0

        # Parameters for ShiTomasi corner detection
        self.feature_params = dict( maxCorners = 100, qualityLevel = 0.3, minDistance = 7, blockSize = 7)

        # Lucas Kanade Optic Flow parameters
        self.lk_params = dict(winSize  = (15,15),
                              maxLevel = 2,
                              criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        # Lucas Kanade Publisher
        self.optic_flow_pub = rospy.Publisher("optic_flow", OpticalFlow, queue_size=10)

        # Raw Image Subscriber
        self.image_sub = rospy.Subscriber('/cf1/camera1/image_raw',Image,self.image_callback)

    def image_callback(self, image):
        try: 
            # if there is an image, then acquire the image, and convert to single channel gray image
            curr_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="mono8")

            # Get time stamp
            secs = image.header.stamp.secs
            nsecs = image.header.stamp.nsecs

            curr_time = float(secs) + float(nsecs)*1e-9

            # If this is the first loop, initialize image matrices
            if self.prev_image is None:
                self.prev_image = curr_image
                self.last_time = curr_time
                self.points_to_track = cv2.goodFeaturesToTrack(self.prev_image, mask=None, **self.feature_params)
                return # skip the rest of this loop

            # get time between images
            dt = curr_time - self.last_time

            # calculate optic flow with lucas kanade
            # see: http://docs.opencv.org/modules/video/doc/motion_analysis_and_object_tracking.html
            # Becareful the datatypes
            new_position_of_tracked_points, status, error = cv2.calcOpticalFlowPyrLK(self.prev_image, curr_image, self.points_to_track.astype(np.float32), None, **self.lk_params)

            # calculate flow field
            flow = new_position_of_tracked_points - self.points_to_track
    
            # draw the flow field
            draw_optic_flow_field(curr_image, self.points_to_track, flow)

            # publish optic flow data to rostopic
            msg = OpticalFlow()
            msg.header.stamp.secs = secs
            msg.header.stamp.nsecs = nsecs
            msg.dt = dt
            msg.x = self.points_to_track[:,0,0]
            msg.y = self.points_to_track[:,0,1]
            msg.vx = flow[:,0,0] # vector in x direction
            msg.vy = flow[:,0,1] # vector in y direction
            self.optic_flow_pub.publish(msg)

            # save current image and time for next loop
            self.prev_image = curr_image
            self.last_time = curr_time

        except CvBridgeError as e:
            print(e)

    def main(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
            cv2.destroyAllWindows()

if __name__ == '__main__': 
    rospy.init_node('optic_flow_calculator', anonymous=True)
    optic_flow_calculator = OpticFlowCalculator()
    optic_flow_calculator.main()