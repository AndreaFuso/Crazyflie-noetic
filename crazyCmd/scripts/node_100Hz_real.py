#! /usr/bin/env python3
import rospy
from crazyflie_simulator.CrazySim import CrazySim
from crazyflie_manager.CrazyManager import *
from crazy_common_py.dataTypes import Vector3
from std_msgs.msg import Empty

if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('node_100Hz', log_level=rospy.DEBUG)

    # Rate definition for the main loop:
    rate = rospy.Rate(5.0)

    # Publisher definition:
    pace_100Hz_pub = rospy.Publisher('/pace_100Hz', Empty, queue_size=1)

    # Continuous loop:
    while not rospy.is_shutdown():
        pace_100Hz_pub.publish(Empty())
        rate.sleep()

    rospy.spin()