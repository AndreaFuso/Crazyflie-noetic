#! /usr/bin/env python3

# Converting ROS image messages to OpenCV images
import cv2

from cv_bridge import CvBridge
bridge = CvBridge()
cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
# 'passthrough' means the destination image encoding will be the same as the image message
# mono8 and bgr8 are the two image encodings expected by most OpenCV functions. 
# mono8: CV_8UC1, grayscale image 
# bgr8: CV_8UC3, color image with blue-green-red color order 


# Converting OpenCV images to ROS image messages

from cv_bridge import CvBridge
bridge = CvBridge()
cv_image = bridge.cv2_to_imgmsg(cv_image, desired_encoding='passthrough')


