#! /usr/bin/env python3
# ROS modules
import rospy
import logging

# Generic modules
import time

# Crazyflie modules
import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

# Crazyflie address:
URI = 'radio://0/80/2M'

logging.basicConfig(level=logging.ERROR)

if __name__ == '__main__':
	# Node creation:
	rospy.init_node('cf_ros_test_decollo')
	
	# Drivers initialization:
	cflib.crtp.init_drivers()
	
	# Crazyflie instantiation:
	with SyncCrazyflie(URI) as scf:
		with MotionCommander(scf) as mc:
			rospy.loginfo("Taking off...")
			time.sleep(2)
			rospy.loginfo("Going up of 5 cm")
			mc.up(0.05)
			rospy.loginfo("Going down of 5 cm")
			mc.down(0.05)
			time.sleep(1)
			rospy.loginfo("Landing...")
	rospy.spin()


