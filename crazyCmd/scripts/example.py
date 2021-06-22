#! /usr/bin/env python3
# ROS modules
import rospy


# CUSTOM MODULES
from crazyflie_simulator.CrazySim import CrazySim
from crazyflie_manager.CrazyManager import *
from crazy_common_py.dataTypes import Vector3


if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('example', log_level=rospy.DEBUG)

    CF1 = CrazySim('cf1', Vector3(1.0, 0.5, 1.0))
    CF2 = CrazySim('cf2', Vector3(0.5, 0.5, 0.2))


    rospy.spin()