#! /usr/bin/env python3
import rospy
from crazyflie_simulator.CrazySim import CrazySim
from crazyflie_manager.CrazyManager import *
from crazy_common_py.dataTypes import Vector3
from std_msgs.msg import Empty

if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('node_500Hz', log_level=rospy.DEBUG)

    rate = rospy.Rate(500.0)

    pace_500Hz_pub = rospy.Publisher('/pace_500Hz', Empty, queue_size=1)

    while not rospy.is_shutdown():
        pace_500Hz_pub.publish(Empty())
        rate.sleep()

    rospy.spin()