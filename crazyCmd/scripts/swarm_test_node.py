#! /usr/bin/env python3
# ROS MODULES
import time
import math
import rospy
import rosbag
import rospkg
import actionlib


if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('swarm_test_node', log_level=rospy.INFO)
    
    rospy.spin()