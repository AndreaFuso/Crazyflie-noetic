#! /usr/bin/env python3
# ROS modules
import rospy
from std_msgs.msg import String

# Generic modules
import logging
import time

# Crazyflie modules
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

# Address of the crazyflie:
URI = 'radio://0/80/2M'

# Setting up logging framework:
logging.basicConfig(level=logging.ERROR)

# Simulation time:
simulationTime = 10

# Node initialization:
rospy.init_node('cr_ros_test_log')

# Publisher definition:
pub = rospy.Publisher('/cf_attitude', String, queue_size=1)

# Message initialization:
message = String()
message.data = ""

#===================================================================================
#
#                       F U N C T I O N S  D E F I N I T I O N S
#
#===================================================================================


def log_stab_callback(timestamp, data, logconf):
    global message, pub
    message.data = str(data)
    pub.publish(message)


def simple_log_async(scf, logconf):
    global simulationTime
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()
    time.sleep(simulationTime)
    logconf.stop()

if __name__ == '__main__':
    # Driver initialization:
    cflib.crtp.init_drivers()

    # Setting up the data to be logged:
    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        simple_log_async(scf, lg_stab)

    rospy.spin()