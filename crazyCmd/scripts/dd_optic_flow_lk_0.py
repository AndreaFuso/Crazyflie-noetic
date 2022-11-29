#! /usr/bin/env python3

# ROS imports
import rospy

# Opencv imports
import cv2

# numpy imports-basic math and matrix manipulation
import numpy as np

# imports for ROS image handling
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

# message imports specific to this package
from crazyflie_messages.msg import OpticalFlow


def draw_optic_flow_field(gray_image, points, flow):
    # gray_image: opencv gray image, e.g. shape = (width, height)
    # points: points at which optic flow is tracked, e.g. shape = (npoints, 1, 2)
    # flow: optic flow field, should be same shape as points

    color_img = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
    # color_red = [0,0,255] # bgr colorspace
    # linewidth = 1
    # for i, point in enumerate(points):
    #     x = point[0,0]
    #     y = point[0,1]
    #     vx = flow[i][0,0]
    #     vy = flow[i][0,1]
    #     cv2.line(color_img, (x,y), (x+vx, y+vy), color_red, linewidth) # draw a red line from the point with vector = [vx, vy] 

    cv2.imshow('optic_flow_field',color_img)
    cv2.waitKey(1)

def define_points_at_which_to_track_optic_flow(image, spacing):
    points_to_track = []
    for x in range(0, image.shape[0], spacing):
        for y in range(0, image.shape[1], spacing):
            new_point = [y, x]
            points_to_track.append(new_point)
    points_to_track = np.array(points_to_track, dtype=np.float32) # note: float32 required for opencv optic flow calculations
    points_to_track = points_to_track.reshape(points_to_track.shape[0], 1, points_to_track.shape[1]) # for some reason this needs to be shape (npoints, 1, 2)
    return points_to_track

class OpticFlowCalculator():
    def __init__(self):
        # Define the source of the images, e.g. rostopic name

        # Initialize image acquisition
        self.bridge = CvBridge()
        self.prev_image = None
        self.last_time = 0
        self.position_x = 0

        # Lucas Kanade Optic Flow parameters
        self.lk_params = dict(winSize  = (15,15),
                              maxLevel = 2,
                              criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        # Lucas Kanade Publisher
        self.optic_flow_pub = rospy.Publisher("optic_flow", OpticalFlow, queue_size=10)
        self.optic_flow_pub_mean = rospy.Publisher("optic_flow_mean_vx", Float32, queue_size=10)

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
                self.points_to_track = define_points_at_which_to_track_optic_flow(curr_image, 10)
                return # skip the rest of this loop

            # get time between images
            dt = curr_time - self.last_time

            # calculate optic flow with lucas kanade
            # see: http://docs.opencv.org/modules/video/doc/motion_analysis_and_object_tracking.html
            new_position_of_tracked_points, status, error = cv2.calcOpticalFlowPyrLK(self.prev_image, curr_image, self.points_to_track, None, **self.lk_params)

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
            msg.vx = flow[:,0,0]
            msg.vy = flow[:,0,1]
            self.optic_flow_pub.publish(msg)

            vx_mean = np.mean(flow[:,0,0])
            self.position_x = self.position_x+vx_mean*dt
            # publish mean optic flow vx data
            self.optic_flow_pub_mean.publish(self.position_x)

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