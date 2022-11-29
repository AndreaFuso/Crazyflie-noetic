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
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry



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

        # Initialize image acquisition
        self.bridge = CvBridge()
        self.prev_image = None
        self.last_time = 0

        # Initialize some parameters
        self.gyr_x = 0
        self.gyr_y = 0
        self.h = 0

        # Initialize current and previous position
        self.prev_x = 0
        self.prev_y = 0


        # Initialize twist info
        self.twist_msg = Twist()
        self.pose_msg = Pose()
        self.visual_odom = Odometry()
        self.visual_odom_pose = Odometry() 

        # Parameters for ShiTomasi corner detection
        self.feature_params = dict( maxCorners = 100, qualityLevel = 0.1, minDistance = 7, blockSize = 7)

        # Lucas Kanade Optic Flow parameters
        self.lk_params = dict(winSize  = (15,15),
                              maxLevel = 2,
                              criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03), 
                              )
                            #   minEigThreshold = 0.0001

        # optical flow twist pub
        self.twist_pub = rospy.Publisher("optical_flow/twist", Twist, queue_size=10)
        self.optical_flow_pub = rospy.Publisher('optical_flow_twist', Odometry, queue_size=10)
        self.pose_pub = rospy.Publisher("optical_flow/pose", Pose, queue_size=10)
        self.optical_flow_pose_pub = rospy.Publisher('optical_flow_pose', Odometry, queue_size=10)

        # Raw Image Subscriber
        self.image_sub = rospy.Subscriber('/cf1/camera1/image_raw',Image,self.image_callback)
        
        # Subscribe to the laser reading collectiion
        rospy.Subscriber("/cf1/laser_reading_tot", Float64MultiArray, self.clbk_laser)

        # Subscribe to the imu gyroscope data
        rospy.Subscriber('/cf1/imu',Imu, self.clbk_imu)

    def clbk_imu(self,msg):
        # get angular velocity from msg
        self.gyr_x = msg.angular_velocity.x
        self.gyr_y = msg.angular_velocity.y

    def clbk_laser(self, msg):
        # Get bottom laser reading from subscribed topic
        self.h = msg.data[5]

    def image_callback(self, image):
        try: 
            # if there is an image, then acquire the image, and convert to single channel gray image
            cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
            curr_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            # curr_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="mono8")

            # Get time stamp
            secs = image.header.stamp.secs
            nsecs = image.header.stamp.nsecs

            curr_time = float(secs) + float(nsecs)*1e-9

            # If this is the first loop, initialize image matrices
            if self.prev_image is None:
                self.prev_image = curr_image
                self.last_time = curr_time
                self.prev_h = self.h
                self.points_to_track = cv2.goodFeaturesToTrack(self.prev_image, mask=None, **self.feature_params)
                return # skip the rest of this loop

            # get time between images
            dt = curr_time - self.last_time

            if dt == 0:
                return

            # calculate optic flow with lucas kanade
            # see: http://docs.opencv.org/modules/video/doc/motion_analysis_and_object_tracking.html
            # Becareful the datatypes
            new_position_of_tracked_points, status, error = cv2.calcOpticalFlowPyrLK(self.prev_image, curr_image, self.points_to_track.astype(np.float32), None, **self.lk_params)

            # calculate flow field
            flow = new_position_of_tracked_points - self.points_to_track
    
            # draw the flow field
            draw_optic_flow_field(curr_image, self.points_to_track, flow)

            # publish optic flow data to rostopic
            # msg = OpticalFlow()
            # msg.header.stamp.secs = secs
            # msg.header.stamp.nsecs = nsecs
            # msg.dt = dt
            # msg.x = self.points_to_track[:,0,0]
            # msg.y = self.points_to_track[:,0,1]
            # msg.vx = flow[:,0,0] # vector in x direction
            # msg.vy = flow[:,0,1] # vector in y direction
            # self.optic_flow_pub.publish(msg)

            # calculate average pixels increment in two direction
            delta_px_mean = np.mean(flow[:,0,0])
            delta_py_mean = np.mean(flow[:,0,1])

            # transform to real world
            theta_py = math.radians(42)
            theta_px = math.radians(42)
            Nx = 350
            Ny = 350
            pitch_rate = self.gyr_y
            roll_rate = self.gyr_x
            h = self.h
            vel_x = h*theta_py*delta_py_mean/(dt*Nx)-h*pitch_rate
            vel_y = h*theta_px*delta_px_mean/(dt*Ny)-h*roll_rate
            vel_z = (self.prev_h-h)/dt

            # publish twist info to /optical_flow/twist
            self.twist_msg.linear.x = vel_x
            self.twist_msg.linear.y = vel_y
            self.twist_msg.linear.z = 0 #vel_z
            self.twist_msg.angular.x = 0
            self.twist_msg.angular.y = 0
            self.twist_msg.angular.z = 0
            self.twist_pub.publish(self.twist_msg)

            # publish twist info with covariance to /optical_flow_twist
            current_time = rospy.get_rostime()
            self.visual_odom.header.frame_id = 'crazyflie_main_body'
            self.visual_odom.header.stamp = current_time
            self.visual_odom.twist.twist.linear.x = vel_x
            self.visual_odom.twist.twist.linear.y = vel_y
            self.visual_odom.twist.twist.linear.z = 0 #vel_z
            self.visual_odom.twist.twist.angular.x = 0.0
            self.visual_odom.twist.twist.angular.y = 0.0
            self.visual_odom.twist.twist.angular.z = 0.0            
            self.visual_odom.twist.covariance[0] = 0.001
            self.visual_odom.twist.covariance[7] = 0.001
            self.visual_odom.twist.covariance[14] = 0.001
            self.visual_odom.twist.covariance[21] = 1000000.0
            self.visual_odom.twist.covariance[28] = 1000000.0
            self.visual_odom.twist.covariance[35] = 1000000.0
            self.optical_flow_pub.publish(self.visual_odom)

            # get the position from velocity
            curr_x = self.prev_x + vel_x*dt
            curr_y = self.prev_y + vel_y*dt
            curr_z = self.h

            # publish the position info to /optical_flow/pose
            self.pose_msg.position.x = curr_x
            self.pose_msg.position.y = curr_y
            self.pose_msg.position.z = curr_z
            self.pose_msg.orientation.x = 0
            self.pose_msg.orientation.y = 0
            self.pose_msg.orientation.z = 0
            self.pose_msg.orientation.w = 0
            self.pose_pub.publish(self.pose_msg)

            # publish the pose info with covariance to /optical_flow_pose
            self.visual_odom_pose.header.frame_id = 'crazyflie_main_body'
            self.visual_odom_pose.header.stamp = current_time
            self.visual_odom_pose.pose.pose.position.x = curr_x
            self.visual_odom_pose.pose.pose.position.y = curr_y
            self.visual_odom_pose.pose.pose.position.z = curr_z
            self.visual_odom_pose.pose.pose.orientation.x = 0
            self.visual_odom_pose.pose.pose.orientation.y = 0
            self.visual_odom_pose.pose.pose.orientation.z = 0
            self.visual_odom_pose.pose.pose.orientation.w = 0
            self.visual_odom_pose.pose.covariance[0] = 0.001
            self.visual_odom_pose.pose.covariance[7] = 0.001
            self.visual_odom_pose.pose.covariance[14] = 0.001
            self.visual_odom_pose.pose.covariance[21] = 1000000.0
            self.visual_odom_pose.pose.covariance[28] = 1000000.0
            self.visual_odom_pose.pose.covariance[35] = 1000000.0
            self.optical_flow_pose_pub.publish(self.visual_odom_pose)

            # save current image and time for next loop
            self.prev_image = curr_image
            self.last_time = curr_time
            self.prev_x = curr_x
            self.prev_y = curr_y
            self.prev_h = curr_z

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