#! /usr/bin/env python3

import rospy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

def clbk_laser(msg):
    value = min(min(msg.ranges[:]),4)
    laser_scan_msg = Float64()
    laser_scan_msg.data = value
    laser_scan_pub.publish(laser_scan_msg)

def main():
    global laser_scan_pub
    rospy.init_node("laser_reading")
    laser_scan_sub = rospy.Subscriber("/cf1/laser/scan",LaserScan,clbk_laser)
    laser_scan_pub = rospy.Publisher("/cf1/laser_scan_z", Float64, queue_size=1)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()