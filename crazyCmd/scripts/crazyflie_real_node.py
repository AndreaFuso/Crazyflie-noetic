#! /usr/bin/env python3
# ROS MODULES
import rospy

# CUSTOM MODULES
from crazy_common_py.dataTypes import Vector3
from crazyflie_drone.CrazyDrone import CrazyDrone

# Function called when the node is shutdown, in order to perform exiting operations:
def exiting_hook():
    global drone
    drone.exit_operations()

if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('crazyflie_real_node', log_level=rospy.DEBUG)

    '''# Extracting rosparam informations (to understand the name and spawn position):
    crazyflie_name = rospy.get_param('crazyflie_spawner_node/name')
    initial_pos = rospy.get_param('crazyflie_spawner_node/initial_position')

    # Spawning the virtual Crazyflie:
    CrazySim(crazyflie_name, Vector3(initial_pos[0], initial_pos[1], initial_pos[2]))'''

    # CrazyDrone instance:
    drone = CrazyDrone('cf1', 'radio://0/80/2M/E7E7E7E7E2', Vector3(0, 0, 0))

    rospy.spin()

    rospy.on_shutdown(exiting_hook)