#! /usr/bin/env python3

import rospy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray

class CollectLaserReading():
    def __init__(self):
        # Initialize parameters
        self.front = 0
        self.back = 0
        self.left = 0
        self.right = 0
        self.top = 0
        self.bottom = 0
        self.tot_msg = Float64MultiArray()

        #Node cycle rate
        self.rate = rospy.Rate(100)

        # Publishers
        self.lase_reading_tot_pub = rospy.Publisher('/cf1/laser_reading_tot', Float64MultiArray, queue_size=1)

         # Subscribers
        rospy.Subscriber("/cf1/front_sensor/scan", LaserScan, self.clbk_laser_front)
        rospy.Subscriber("/cf1/back_sensor/scan", LaserScan, self.clbk_laser_back)
        rospy.Subscriber("/cf1/left_sensor/scan",LaserScan,self.clbk_laser_left)
        rospy.Subscriber("/cf1/right_sensor/scan",LaserScan,self.clbk_laser_right)
        rospy.Subscriber("/cf1/top_sensor/scan",LaserScan,self.clbk_laser_top)
        rospy.Subscriber("/cf1/laser/scan",LaserScan,self.clbk_laser_bottom)

    # Callbacks for the subscribers
    def clbk_laser_front(self, msg):
        self.front = min(min(msg.ranges[14:17]),4)

    def clbk_laser_back(self, msg):
        self.back = min(min(msg.ranges[14:17]),4)

    def clbk_laser_left(self, msg):
        self.left = min(min(msg.ranges[14:17]),4)

    def clbk_laser_right(self, msg):
        self.right = min(min(msg.ranges[14:17]),4)

    def clbk_laser_top(self, msg):
        self.top = min(min(msg.ranges[14:17]),4)

    def clbk_laser_bottom(self, msg):
        self.bottom = min(min(msg.ranges[14:17]),4)

    # To publish the message
    def publish_action(self):
        while not rospy.is_shutdown():
            # rospy.loginfo("front:{}, back:{}, left:{}, right:{}, top:{}, bottom:{}".format(self.front, self.back, self.left, self.right, self.top, self.bottom))
            self.tot_msg.data = [self.front, self.back, self.left, self.right, self.top, self.bottom]
            self.lase_reading_tot_pub.publish(self.tot_msg)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node("collect_laser_reading")
    my_node = CollectLaserReading()
    my_node.publish_action()