#! /usr/bin/env python3
# ROS MODULES
import time

import rospy

# CUSTOM MODULES
from crazy_common_py.dataTypes import Vector3
from crazy_common_py.common_functions import standardNameList

from crazyflie_swarm.CrazyPyramidSwarmSim import CrazyPyramidSwarmSim

if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('pyramid_swarm_node', log_level=rospy.ERROR)

    # Extracting rosparam informations (to understand the number of crazyflies):
    number_of_cfs = rospy.get_param('pyramid_swarm_node/cfs_number')
    number_of_levels = rospy.get_param('pyramid_swarm_node/levels')
    vertical_offset = rospy.get_param('pyramid_swarm_node/vertical_offset')

    # Generate a standard list of names:
    cf_names = standardNameList(number_of_cfs)

    # Instantiate a swarm:
    swarm = CrazyPyramidSwarmSim(cf_names, number_of_levels, vertical_offset)

    time.sleep(10)


    rospy.spin()