#! /usr/bin/env python3
# ROS MODULES
import time
import rospy

# CUSTOM MODULES
from crazy_common_py.dataTypes import Vector3
from crazy_common_py.common_functions import standardNameList
from crazyflie_swarm.CrazySwarmReal import CrazySwarmReal

from cflib.crazyflie.swarm import Swarm, CachedCfFactory, _Factory



# def exiting_hook():
#     swarm.__swarm.close_links()


if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('swarm_real_node', log_level=rospy.DEBUG)

    # Extracting rosparam informations (to understand the number of crazyflies):
    number_of_cfs = rospy.get_param('swarm_node/cfs_number')

    # Generate a standard list of names:
    cf_names = standardNameList(number_of_cfs)
    print('cf_names are: ', cf_names)

    # Instantiation of CrazySwarmReal
    swarm = CrazySwarmReal(cf_names)

    time.sleep(10)

    # print(cf_names)
    # print(swarm.cf_names)
    # print(type(swarm.takeoff_act_clients[0]))

    rospy.spin()


    # rospy.on_shutdown(exiting_hook)