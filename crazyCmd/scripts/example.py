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
'''
PROBLEMI:
    1) Quando ne spawno due uno inizia ad oscillare, possibili cause:
        - computer non li gestisce (poco probabile);
        - in CrazySim uso due volte roslaunch in __start_gazebo_controllers: provare a fare questa operazione del manager dello sciame;
        - il movimento con go_to non funziona come un'azione?
'''

if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('example', log_level=rospy.INFO)

    pos1 = Vector3(1.0, 0.0, 0.2)
    pos2 = Vector3(1.0, 2.0, 0.2)
    pos3 = Vector3(1.0, 4.0, 0.2)
    pos4 = Vector3(1.0, 6.0, 0.2)

    pos_sel = pos1
    deltax = 1.5
    deltay = 1.5
    deltaz = 2.0

    CF1 = CrazySim('cf1', pos_sel)
    CF2 = CrazySim('cf2', pos2)
    #CF3 = CrazySim('cf3', pos3)
    #CF4 = CrazySim('cf4', pos4)

    time.sleep(5)
    CF1.motion_commander.takeoff_actn()
    CF2.motion_commander.takeoff_actn()
    #CF3.motion_commander.takeoff_actn()
    #CF4.motion_commander.takeoff_actn()

    time.sleep(3)
    CF1.motion_commander.go_to(Vector3(pos_sel.x + deltax, pos_sel.y + deltay, pos_sel.z + deltaz), yaw=30.0)
    CF2.motion_commander.go_to(Vector3(pos2.x + deltax, pos2.y + deltay, pos2.z + deltaz), yaw=30.0)

    '''CF1.motion_commander.takeoff_srv()
    CF2.motion_commander.takeoff_srv()
    CF3.motion_commander.takeoff_srv()
    CF4.motion_commander.takeoff_srv()'''

    '''
    Se fossero i cicli while nelle actions lo vedrei nel carico della cpu?
    '''

    '''time.sleep(10)
    CF1.motion_commander.go_to(Vector3(1.5, 0.5, 0.2), yaw=30.0)
    #CF2.motion_commander.go_to(Vector3(1.5, 1.0, 0.2), yaw=30.0)

    time.sleep(5)
    CF1.motion_commander.go_to(Vector3(1.5, 0.5, 0.2), yaw=90.0)
    #CF2.motion_commander.go_to(Vector3(1.5, 1.0, 0.2), yaw=90.0)

    time.sleep(5)
    CF1.motion_commander.go_to(Vector3(1.5, 0.5, 0.2), yaw=-30.0)
    #CF2.motion_commander.go_to(Vector3(1.5, 1.0, 0.2), yaw=-30.0)

    time.sleep(5)
    distance = 2.0
    x1 = 1.5 + distance * math.cos(deg2rad(30.0))
    y1 = 0.5 + distance * math.sin(deg2rad(30.0))
    x2 = 1.5 + distance * math.cos(deg2rad(30.0))
    y2 = 1.0 + distance * math.sin(deg2rad(30.0))

    CF1.motion_commander.go_to(Vector3(x1, y1, 0.2), yaw=30.0)
    #CF2.motion_commander.go_to(Vector3(x2, y2, 0.2), yaw=30.0)'''

    '''time.sleep(5)
    CF1.motion_commander.go_to(Vector3(1.0, 1.0, 0.2))

    time.sleep(5)
    CF1.motion_commander.go_to(Vector3(1.0, 0.5, 0.2))'''

    rospy.spin()
