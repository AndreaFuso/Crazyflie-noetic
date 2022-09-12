#! /usr/bin/env python3

import rospy

from sensor_msgs.msg import LaserScan

def clbk_laser(msg):
    regions = min(msg.ranges[:])
    # regions = []
    # for i in range(0,50,10):
    #     value = min(min(msg.ranges[i:i+9]),10)
    #     regions.append(value)
    # regions = [
    #     min(min(msg.ranges[0:143]),10),
    #     min(min(msg.ranges[144:287]),10),
    #     min(min(msg.ranges[288:431]),10),
    #     min(min(msg.ranges[432:575]),10),
    #     min(min(msg.ranges[576:713]),10),
    # ]
    rospy.loginfo(regions)

def main():
    rospy.init_node("laser_reading")
    sub = rospy.Subscriber("/cf1/laser/scan",LaserScan,clbk_laser)
    rospy.spin()

if __name__ == '__main__':
    main()