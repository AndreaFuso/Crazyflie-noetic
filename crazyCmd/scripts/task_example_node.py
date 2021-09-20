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

import roslaunch

if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('task_example_node', log_level=rospy.INFO)

    name1 = 'cf1'
    pos1 = [0.0, 0.0, 0.2]

    name2 = 'cf2'
    pos2 = [0.0, 1.0, 0.2]

    rospy.set_param('/' + name1 + '/crazyflie_spawner_node/name', name1)
    rospy.set_param('/' + name1 + '/crazyflie_spawner_node/initial_position', pos1)

    '''rospy.set_param('/' + name2 + '/crazyflie_spawner_node/name', name2)
    rospy.set_param('/' + name2 + '/crazyflie_spawner_node/initial_position', pos2)'''

    ns1 = '/' + name1
    node1 = roslaunch.core.Node(package='crazyCmd', node_type='crazyflie_spawner_node.py', name="crazyflie_spawner_node", namespace=ns1)
    #node2 = roslaunch.core.Node(package='crazyCmd', node_type='crazyflie_spawner_node', namespace='/' + name2)

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    process = launch.launch(node1)
