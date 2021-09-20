#! /usr/bin/env python3
# ROS MODULES
import rospy
import roslaunch
import rospkg
# GAZEBO MODULES
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest

import sys
import xacro
import subprocess
import time

# CUSTOM MODULES
from crazy_common_py.dataTypes import Vector3, GazeboIMU
from crazy_common_py.constants import *
from crazyflie_simulator.pid import *
from crazyflie_simulator.FlightControllerSimFirmwr import FlightControllerSimFirmwr
from crazyflie_simulator.MyFlightControllerFirmwr import MyFlightControllerFirmwr
from crazyflie_simulator.FlightControllerSimCustom import FlightControllerCustom
from crazyflie_simulator.CrazySim import CrazySim

# OTHER MODULES
import os

if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('crazyflie_spawner_node', log_level=rospy.DEBUG)

    # Extracting rosparam informations (to understand the name and spawn position):
    crazyflie_name = rospy.get_param('crazyflie_spawner_node/name')
    initial_pos = rospy.get_param('crazyflie_spawner_node/initial_position')

    # Spawning a virtual Crazyflie:
    CrazySim(crazyflie_name, Vector3(initial_pos[0], initial_pos[1], initial_pos[2]))

    rospy.spin()