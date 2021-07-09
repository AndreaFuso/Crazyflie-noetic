#! /usr/bin/env python3
# ROS modules
import rospy

import time

# CUSTOM MODULES
from crazyflie_simulator.CrazySim import CrazySim
from crazyflie_manager.CrazyManager import *
from crazy_common_py.dataTypes import Vector3


if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('example', log_level=rospy.INFO)

    CF1 = CrazySim('cf1', Vector3(1.0, 0.5, 0.2))
    CF2 = CrazySim('cf2', Vector3(1.0, 1.5, 0.2))
    time.sleep(3)
    CF1.motion_commander.takeoff_srv()
    CF2.motion_commander.takeoff_srv()
    CF1.motion_commander.go_to(Vector3(5.0, 5.0, 2.0), 90.0)

    rospy.spin()