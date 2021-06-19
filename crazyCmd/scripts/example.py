#! /usr/bin/env python3
# ROS modules
import rospy

# CUSTOM MODULES
from crazyflie_simulator.CrazySim import CrazySim
from crazyflie_manager.CrazyManager import *


if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('example', log_level=rospy.DEBUG)

    '''# Data of agent crazyflie:
    agent = CfAgent('radio://0/80/2M/E7E7E7E7E7', 'cf1')
    agent.add_deck("bcFlow2")
    agent.add_deck("bcZRanger2")
    agent.add_log_item('stabilizer.roll', 'float')
    agent.add_log_item('stabilizer.pitch', 'float')
    agent.add_log_item('stabilizer.yaw', 'float')

    # Driver initialization:
    cflib.crtp.init_drivers()

    # CrazyflieManager instantiation:
    cf1 = CrazyflieManager(agent)

    rospy.on_shutdown(shutdown_operations)'''

    CrazySim('cf1')

    rospy.spin()