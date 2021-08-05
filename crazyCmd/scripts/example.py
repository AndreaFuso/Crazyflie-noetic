#! /usr/bin/env python3
# ROS modules
import rospy

import time

# CUSTOM MODULES
from crazyflie_simulator.CrazySim import CrazySim
from crazyflie_manager.CrazyManager import *
from crazy_common_py.dataTypes import Vector3
from crazy_common_py.common_functions import deg2rad
import math

if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('example', log_level=rospy.INFO)

    CF1 = CrazySim('cf1', Vector3(1.0, 0.5, 0.2))
    time.sleep(3)
    CF1.motion_commander.takeoff_actn()

    time.sleep(5)
    CF1.motion_commander.go_to(Vector3(1.5, 0.5, 0.2), yaw=30.0)

    '''time.sleep(5)
    CF1.motion_commander.go_to(Vector3(1.5, 0.5, 0.2), yaw=90.0)

    time.sleep(5)
    CF1.motion_commander.go_to(Vector3(1.5, 0.5, 0.2), yaw=-30.0)

    time.sleep(5)
    CF1.motion_commander.go_to(Vector3(1.5, 0.5, 0.2), yaw=30.0)

    time.sleep(5)
    distance = 2.0
    x = 1.5 + distance * math.cos(deg2rad(30.0))
    y = 0.5 + distance * math.sin(deg2rad(30.0))
    CF1.motion_commander.go_to(Vector3(x, y, 0.2), yaw=30.0)'''

    '''time.sleep(5)
    CF1.motion_commander.go_to(Vector3(1.0, 1.0, 0.2))

    time.sleep(5)
    CF1.motion_commander.go_to(Vector3(1.0, 0.5, 0.2))'''

    rospy.spin()
