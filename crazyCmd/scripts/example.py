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

    deltax = 0.5
    deltay = 0.0

    CF1 = CrazySim('cf1', Vector3(1.0, 0.5, 0.2))
    CF2 = CrazySim('cf2', Vector3(1.0 + deltax, 0.5 + deltay, 0.2))

    CF1.motion_commander.takeoff()
    CF2.motion_commander.takeoff()

    time.sleep(5)
    CF1.motion_commander.go_to(Vector3(1.0, 0.5, 0.5))
    CF2.motion_commander.go_to(Vector3(1.0 + deltax, 0.5 + deltay, 0.5))
    time.sleep(3)
    CF1.motion_commander.go_to(Vector3(1.5, 0.5, 0.5))
    CF2.motion_commander.go_to(Vector3(1.5 + deltax, 0.5 + deltay, 0.5))
    time.sleep(3)
    CF1.motion_commander.go_to(Vector3(1.5, 1.0, 0.5))
    CF2.motion_commander.go_to(Vector3(1.5 + deltax, 1.0 + deltay, 0.5))
    time.sleep(3)
    CF1.motion_commander.go_to(Vector3(0.5, 1.0, 0.5))
    CF2.motion_commander.go_to(Vector3(0.5 + deltax, 1.0 + deltay, 0.5))
    time.sleep(3)
    CF1.motion_commander.go_to(Vector3(0.5, 0.5, 0.5))
    CF2.motion_commander.go_to(Vector3(0.5 + deltax, 0.5 + deltay, 0.5))
    time.sleep(3)
    CF1.motion_commander.go_to(Vector3(2.0, 2.0, 2.0))
    CF2.motion_commander.go_to(Vector3(2.0 + deltax, 2.0 + deltay, 2.0))
    time.sleep(5)
    CF1.motion_commander.prepareLanding()
    CF2.motion_commander.prepareLanding()

    '''CF1.motor_controller.sendManualCommand(39000, 0, 0, 0)
    time.sleep(1)
    CF1.motor_controller.sendManualCommand(38200, 0, 0, -1000)
    time.sleep(0.5)'''
    '''
        ROLL:
            r > 0 => +x rotation
            r < 0 => -x rotation
        PITCH:
            p > 0 => -y rotation
            p < 0 = > +y rotation
        YAW:
            y > 0 => -z rotation
            y < 0 => +z rotation
    '''
    '''rate = rospy.Rate(500)
    while not rospy.is_shutdown():
        CF1.motor_controller.sendManualCommand(38200, 0, -50, 1000)
        CF1.motor_controller.sendManualCommand(38200, 0, -50, -1000)

    rospy.spin()'''
    '''thrustCmd = 38180 #38180
    rate = rospy.Rate(500)
    cont = 0
    time.sleep(5)
    print("ENTRO NEL CICLO")
    while not rospy.is_shutdown():
        if cont < 20000:
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