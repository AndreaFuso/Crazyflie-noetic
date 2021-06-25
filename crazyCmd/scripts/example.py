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

    CF1 = CrazySim('cf1', Vector3(1.0, 0.5, 0.2))
    #CF2 = CrazySim('cf2', Vector3(0.5, 0.5, 0.2))

    CF1.motion_commander.go_to(Vector3(1.0, 0.5, 0.5))

    '''thrustCmd = 38180
    rate = rospy.Rate(500)
    cont = 0
    time.sleep(5)
    print("ENTRO NEL CICLO")
    while not rospy.is_shutdown():
        if cont < 3000:
            CF1.motor_controller.M1.sendThrustCommand(thrustCmd)
            CF1.motor_controller.M2.sendThrustCommand(thrustCmd)
            CF1.motor_controller.M3.sendThrustCommand(thrustCmd)
            CF1.motor_controller.M4.sendThrustCommand(thrustCmd)
        else:
            print("SPENGO I MOTORI")
            CF1.motor_controller.M1.stopMotor()
            CF1.motor_controller.M2.stopMotor()
            CF1.motor_controller.M3.stopMotor()
            CF1.motor_controller.M4.stopMotor()
        print(cont)
        cont += 1
        rate.sleep()'''

    rospy.spin()