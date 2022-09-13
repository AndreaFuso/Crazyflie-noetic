#! /usr/bin/env python3

import rospy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

def clbk_laser_front(msg):
    value = min(min(msg.ranges[:]),4)
    laser_scan_msg = Float64()
    laser_scan_msg.data = value
    front_pub.publish(laser_scan_msg)

def clbk_laser_back(msg):
    value = min(min(msg.ranges[:]),4)
    laser_scan_msg = Float64()
    laser_scan_msg.data = value
    back_pub.publish(laser_scan_msg)

def clbk_laser_left(msg):
    value = min(min(msg.ranges[:]),4)
    laser_scan_msg = Float64()
    laser_scan_msg.data = value
    left_pub.publish(laser_scan_msg)

def clbk_laser_right(msg):
    value = min(min(msg.ranges[:]),4)
    laser_scan_msg = Float64()
    laser_scan_msg.data = value
    right_pub.publish(laser_scan_msg)

def clbk_laser_top(msg):
    value = min(min(msg.ranges[:]),4)
    laser_scan_msg = Float64()
    laser_scan_msg.data = value
    top_pub.publish(laser_scan_msg)
    
def main():
    global front_pub, back_pub, left_pub, right_pub, top_pub
    rospy.init_node("multiranger_reading")
    front_sub = rospy.Subscriber("/cf1/front_sensor/scan",LaserScan,clbk_laser_front)
    front_pub = rospy.Publisher("/cf1/laser_scan_front", Float64, queue_size=1)

    back_sub = rospy.Subscriber("/cf1/back_sensor/scan",LaserScan,clbk_laser_back)
    back_pub = rospy.Publisher("/cf1/laser_scan_back", Float64, queue_size=1)

    left_sub = rospy.Subscriber("/cf1/left_sensor/scan",LaserScan,clbk_laser_left)
    left_pub = rospy.Publisher("/cf1/laser_scan_left", Float64, queue_size=1)

    right_sub = rospy.Subscriber("/cf1/right_sensor/scan",LaserScan,clbk_laser_right)
    right_pub = rospy.Publisher("/cf1/laser_scan_right", Float64, queue_size=1)

    top_sub = rospy.Subscriber("/cf1/top_sensor/scan",LaserScan,clbk_laser_top)
    top_pub = rospy.Publisher("/cf1/laser_scan_top", Float64, queue_size=1)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()