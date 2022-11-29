#! /usr/bin/env python3

# Through this code, we can make our crazyflie to fly following a yellow line, but it doesn't work

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from crazyflie_messages.msg import Position

import cv2
import numpy as np

yellow = np.uint8([[[241,249,6]]])
hsv_yellow = cv2.cvtColor(yellow, cv2.COLOR_BGR2HSV)
print(hsv_yellow)

class LineFollower():
    def __init__(self):
        self.image_sub = rospy.Subscriber('/cf1/camera1/image_raw', Image, self.camera_callback)
        self.bridge_object = CvBridge()
        self.cmd_vel_pub = rospy.Publisher('/cf1/cmd_vel', Position, queue_size=1)

    def camera_callback(self, data):
        try:
            # bgr8 is the default OpenCV encoding 
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)
        # to reduce the dimension of the images
        height, width, channels = cv_image.shape
        descentre = 160
        rows_to_watch = 60
        crop_img = cv_image
        # crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]

        # make the color detection independent of light conditions, converted to another image type
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        # upper_yellow = np.array([70,48,255])
        # lower_yellow = np.array([50,28,245])

        upper_yellow = np.array([99,249,255])
        lower_yellow = np.array([79,249,255])

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Compute the centroid of the blob, which is the line we want to follow
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2

        error_x = cx - width / 2
        speed_cmd = Position()
        speed_cmd.desired_velocity.x = 0.2
        speed_cmd.desired_yaw_rate = -error_x / 100
        
        self.cmd_vel_pub.publish(speed_cmd)

def main():
    rospy.init_node('line_following', anonymous=True)
    line_follower = LineFollower()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')

if __name__ == '__main__':
    main()



