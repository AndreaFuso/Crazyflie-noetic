#! /usr/bin/env python3
import logging
import time
import rospy
import math

import cflib.crtp 
from cflib.crazyflie import Crazyflie 
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie 

from cflib.crazyflie.log import LogConfig 
from cflib.crazyflie.syncLogger import SyncLogger 
from cflib.utils import uri_helper
from cflib.positioning.motion_commander import MotionCommander

from crazyflie_messages.msg import CrazyflieState

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E2')

# Default parameters
DEFAULT_HEIGHT = 0.5

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# def simple_connect():

#     print("Yeah, I'm connected! :D")
    # time.sleep(3)
    # print("Now I will disconnect :'(")

######## Asynchronous state logging ########
# for the pose
def log_state_pose_callback(timestamp, data, logconf_state_pose):
    print('[%d][%s]: %s' % (timestamp, logconf_state_pose.name, data))
    pose_msg = CrazyflieState()
    pose_msg.name = 'cf1'
    pose_msg.position.x = data['stateEstimate.x']
    pose_msg.position.y = data['stateEstimate.y']
    pose_msg.position.z = data['stateEstimate.z']
    pose_msg.orientation.roll = data['stateEstimate.roll']
    pose_msg.orientation.pitch = data['stateEstimate.pitch']
    pose_msg.orientation.yaw = data['stateEstimate.yaw']
    pose_pub.publish(pose_msg)

def state_pose_log_async(scf,logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_state_pose_callback)
    logconf.start()
    time.sleep(120)
    logconf.stop()

# for the twist
def log_state_twist_callback(timestamp, data, logconf_state_twist):
    print('[%d][%s]: %s' % (timestamp, logconf_state_twist.name, data))
    twist_msg = CrazyflieState()
    twist_msg.name = 'cf1'
    twist_msg.velocity.x = data['stateEstimate.vx']
    twist_msg.velocity.y = data['stateEstimate.vy']
    twist_msg.velocity.z = data['stateEstimate.vz']
    twist_msg.rotating_speed.x = data['stateEstimateZ.rateRoll']/1000*180/math.pi #[degree/s]
    twist_msg.rotating_speed.y = data['stateEstimateZ.ratePitch']/1000*180/math.pi
    twist_msg.rotating_speed.z = data['stateEstimateZ.rateYaw']/1000*180/math.pi
    twist_pub.publish(twist_msg)

def state_twist_log_async(scf,logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_state_twist_callback)
    logconf.start()
    time.sleep(120)
    logconf.stop()

if __name__ == '__main__':
    global pose_pub, twist_pub
    rospy.init_node('data_record')

    pose_pub = rospy.Publisher("/state_pose_real",CrazyflieState , queue_size=1)
    twist_pub = rospy.Publisher("/state_twist_real",CrazyflieState , queue_size=1)

    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    logconf_pose = LogConfig(name='StatePose', period_in_ms=10)
    logconf_pose.add_variable('stateEstimate.x', 'float') #[m]
    logconf_pose.add_variable('stateEstimate.y', 'float')
    logconf_pose.add_variable('stateEstimate.z', 'float')
    logconf_pose.add_variable('stateEstimate.roll', 'float') #[degree]
    logconf_pose.add_variable('stateEstimate.pitch', 'float')
    logconf_pose.add_variable('stateEstimate.yaw', 'float')
    
    logconf_twist = LogConfig(name='StateTwist', period_in_ms=10)
    logconf_twist.add_variable('stateEstimate.vx', 'float') #[m/s]
    logconf_twist.add_variable('stateEstimate.vy', 'float')
    logconf_twist.add_variable('stateEstimate.vz', 'float')
    logconf_twist.add_variable('stateEstimateZ.rateRoll', 'int16_t') # [milliradians/s]
    logconf_twist.add_variable('stateEstimateZ.ratePitch', 'int16_t')
    logconf_twist.add_variable('stateEstimateZ.rateYaw', 'int16_t')


    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        # simple_connect()
        
        ######## Synchronous state logging ########
        # state_pose_log()
        # state_twist_log()

        # takeoff()
        ######## Asynchronous state logging ########
        state_pose_log_async(scf,logconf_pose)
        state_twist_log_async(scf,logconf_twist)
        
        






######## Synchronous state logging ########

# def state_pose_log():
#     with SyncLogger(scf, logconf_pose) as logger:
#         for log_entry in logger:
#             timestamp = log_entry[0]
#             data = log_entry[1]
#             logconf_name = log_entry[2]

#             print('[%d][%s]: %s' % (timestamp, logconf_name, data))

#             break

# def state_twist_log():
#      with SyncLogger(scf, logconf_twist) as logger:
#         for log_entry in logger:
#             timestamp = log_entry[0]
#             data = log_entry[1]
#             logconf_name = log_entry[2]

#             print('[%d][%s]: %s' % (timestamp, logconf_name, data))

#             break


