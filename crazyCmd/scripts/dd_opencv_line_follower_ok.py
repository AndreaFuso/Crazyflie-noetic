#! /usr/bin/env python3

# Through this code, we can make our crazyflie to fly following a yellow line,
# it works but it still needs to tune the velocity, yaw rate parameters etc

import math
import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from crazyflie_messages.msg import Position

class LineFollower():
    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber('/cf1/camera1/image_raw', Image, self.camera_callback)
        self.cmd_vel_pub = rospy.Publisher('/cf1/cmd_vel', Position, queue_size=1)

    def camera_callback(self, data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except CvBridgeError as e:
            print(e)
            
        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(1) # this will display a frame for at least 1ms, and then will be automatically closed
        height, width, channel = cv_image.shape
        descentre = 87
        rows_to_watch = 20
        # crop_img = cv_image[(int(height)/2+int(descentre)):(int(height)/2+int(descentre)+int(rows_to_watch))][1:int(width]
        
        crop_img = cv_image[262:282][1:350]
        # cv2.imshow("Crop image window", crop_img)
        # cv2.waitKey(1)

        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # my yellow color is BGR [[[44 124 121]]]

        # yellow = np.uint8([[[44,124,121]]])
        # hsv_yellow = cv2.cvtColor(yellow, cv2.COLOR_BGR2HSV)
        # print(hsv_yellow)  #[[[31 165 124]]]

        lower_yellow = np.array([20, 60, 60])
        upper_yellow = np.array([65, 255, 255])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        res = cv2.bitwise_and(crop_img, crop_img, mask= mask)

        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cx, cy = height/2, width/2

        cv2.circle(res, (int(cx),int(cy)), 10, (0,0,255), -1)
        cv2.imshow("Original", cv_image)
        cv2.imshow("HSV", hsv)
        cv2.imshow("MASK", mask)
        cv2.imshow("RES", res)

        cv2.waitKey(1)

        error_x = cx - width/2
        speed_cmd = Position()
        speed_cmd.desired_velocity.x = 0.2
        speed_cmd.desired_yaw_rate = -error_x/100
        rospy.loginfo("ANGULAR VALUE SENT ===>"+str(speed_cmd.desired_yaw_rate))
        self.cmd_vel_pub.publish(speed_cmd)

def main():
    line_follower_object = LineFollower()
    rospy.init_node('line_following_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()




