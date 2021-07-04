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
    rospy.init_node('test_bench', log_level=rospy.INFO)

    CF1 = CrazySim('cf1', Vector3(), True)
