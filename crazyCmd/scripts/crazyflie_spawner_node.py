#! /usr/bin/env python3
# ROS MODULES
import rospy

# CUSTOM MODULES
from crazy_common_py.dataTypes import Vector3
from crazyflie_simulator.CrazySim import CrazySim

if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('crazyflie_spawner_node', log_level=rospy.DEBUG)

    # Extracting rosparam informations (to understand the name and spawn position):
    crazyflie_name = rospy.get_param('crazyflie_spawner_node/name')
    initial_pos = rospy.get_param('crazyflie_spawner_node/initial_position')

    # Spawning a virtual Crazyflie:
    CrazySim(crazyflie_name, Vector3(initial_pos[0], initial_pos[1], initial_pos[2]))

    rospy.spin()