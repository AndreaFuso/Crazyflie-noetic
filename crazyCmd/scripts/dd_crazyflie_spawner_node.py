#! /usr/bin/env python3
# ROS MODULES
import rospy

# CUSTOM MODULES
from crazy_common_py.dataTypes import Vector3
## DD: from dataTypes module inside crazy_commom_py package, import Vector3 class
from crazyflie_simulator.DdCrazySim import CrazySim
## DD: from <module> CrazySim inside <crazyflie_simulator> package, import CrazySim class

if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('crazyflie_spawner_node', log_level=rospy.ERROR)

    # Extracting rosparam informations (to understand the name and spawn position):
    crazyflie_name = rospy.get_param('crazyflie_spawner_node/name')
    initial_pos = rospy.get_param('crazyflie_spawner_node/initial_position')
    ## DD: get parameter value from parameter server, here rospy.get_param will return xxx/name and xxx/initial_position respectively

    # Spawning the virtual Crazyflie:
    CrazySim(crazyflie_name, Vector3(initial_pos[0], initial_pos[1], initial_pos[2]))
    ## DD: use CrazySim class to spawn the Cf

    rospy.spin()