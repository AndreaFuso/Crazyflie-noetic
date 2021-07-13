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

    #CF1 = CrazySim('cf1', Vector3(1.0, 0.5, 0.2))
    CF2 = CrazySim('cf2', Vector3(1.0, 1.5, 0.2))
    time.sleep(3)
    #CF1.motion_commander.takeoff_actn()
    CF2.motion_commander.takeoff_actn()
    time.sleep(5)
    #CF1.motion_commander.turn_left(45)
    CF2.motion_commander.turn_left(90)
    time.sleep(3)
    #CF1.motion_commander.forward(1.0, velocity=0.2)
    CF2.motion_commander.forward(1.0, velocity=1.0)
    time.sleep(8)
    #CF1.motion_commander.land_actn()
    CF2.motion_commander.land_actn()



    rospy.spin()
