#! /usr/bin/env python3
import logging
import time
import sys
import rospy
import math
from threading import Event

import cflib.crtp 
from cflib.crazyflie import Crazyflie 
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie 

from cflib.crazyflie.log import LogConfig 
from cflib.crazyflie.syncLogger import SyncLogger 
from cflib.utils import uri_helper
from cflib.positioning.motion_commander import MotionCommander

from crazyflie_messages.msg import CrazyflieState
from sensor_msgs.msg import LaserScan

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E2')

deck_attached_event = Event()

# Default parameters
DEFAULT_HEIGHT = 0.3

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')

def takeoff(scf):
    with MotionCommander(scf,default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(30)
        mc.stop

def move_rectangle(scf):
    with MotionCommander(scf,default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(2)
        mc.start_forward(0.2)
        time.sleep(5)
        mc.start_turn_left(rate=18)
        time.sleep(5)
        mc.start_forward(0.2)
        time.sleep(5)
        mc.start_turn_left(rate=18)
        time.sleep(5)
        mc.start_forward(0.2)
        time.sleep(5)
        mc.start_turn_left(rate=18)
        time.sleep(5)
        mc.start_forward(0.2)
        time.sleep(5)
        mc.start_turn_left(rate=18)
        time.sleep(5)
        mc.stop()

def log_state_pose_callback(timestamp, data, logconf_pose):
    print('[%d][%s]: %s' % (timestamp, logconf_pose.name, data))
    pose_msg = CrazyflieState()
    pose_msg.name = 'cf1'
    pose_msg.position.x = data['stateEstimate.x']
    pose_msg.position.y = data['stateEstimate.y']
    pose_msg.position.z = data['stateEstimate.z']
    pose_msg.orientation.roll = data['stateEstimate.roll']
    pose_msg.orientation.pitch = data['stateEstimate.pitch']
    pose_msg.orientation.yaw = data['stateEstimate.yaw']
    pose_pub.publish(pose_msg)

def log_state_twist_callback(timestamp, data, logconf_twist):
    print('[%d][%s]: %s' % (timestamp, logconf_twist.name, data))
    twist_msg = CrazyflieState()
    twist_msg.name = 'cf1'
    twist_msg.velocity.x = data['stateEstimate.vx']
    twist_msg.velocity.y = data['stateEstimate.vy']
    twist_msg.velocity.z = data['stateEstimate.vz']
    twist_msg.rotating_speed.x = data['stateEstimateZ.rateRoll']*0.001*180/(math.pi) #[degree/s]
    twist_msg.rotating_speed.y = data['stateEstimateZ.ratePitch']*0.001*180/(math.pi)
    twist_msg.rotating_speed.z = data['stateEstimateZ.rateYaw']*0.001*180/(math.pi)
    twist_pub.publish(twist_msg)

def log_range_callback(timestamp, data, logconf_range):
    print('[%d][%s]: %s' % (timestamp, logconf_range.name, data))
    laser_msg = LaserScan()
    laser_msg.ranges = data['range.front']*0.01
    twist_pub.publish(laser_msg)
    
if __name__ == '__main__':
    rospy.init_node('hovering')
    global pose_pub, twist_pub, laser_pub

    pose_pub = rospy.Publisher("/state_pose_real",CrazyflieState , queue_size=1)
    twist_pub = rospy.Publisher("/state_twist_real",CrazyflieState , queue_size=1)
    laser_pub = rospy.Publisher("/laser_scan_real",LaserScan, queue_size=1)

    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',cb=param_deck_flow)
        time.sleep(1)

        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        logconf_pose = LogConfig(name='StatePose', period_in_ms=10)
        logconf_pose.add_variable('stateEstimate.x', 'float') #[m]
        logconf_pose.add_variable('stateEstimate.y', 'float')
        logconf_pose.add_variable('stateEstimate.z', 'float')
        logconf_pose.add_variable('stateEstimate.roll', 'float') #[degree]
        logconf_pose.add_variable('stateEstimate.pitch', 'float')
        logconf_pose.add_variable('stateEstimate.yaw', 'float')
        scf.cf.log.add_config(logconf_pose)
        logconf_pose.data_received_cb.add_callback(log_state_pose_callback)

        logconf_twist = LogConfig(name='StateTwist', period_in_ms=10)
        logconf_twist.add_variable('stateEstimate.vx', 'float') #[m/s]
        logconf_twist.add_variable('stateEstimate.vy', 'float')
        logconf_twist.add_variable('stateEstimate.vz', 'float')
        logconf_twist.add_variable('stateEstimateZ.rateRoll', 'int16_t') # [milliradians/s]
        logconf_twist.add_variable('stateEstimateZ.ratePitch', 'int16_t')
        logconf_twist.add_variable('stateEstimateZ.rateYaw', 'int16_t')
        scf.cf.log.add_config(logconf_twist)
        logconf_twist.data_received_cb.add_callback(log_state_twist_callback)

        logconf_range = LogConfig(name='Range', period_in_ms=10)
        logconf_range.add_variable('range.front', 'uint16_t') #[mm]
        logconf_range.add_variable('range.back', 'uint16_t')
        logconf_range.add_variable('range.left', 'uint16_t')
        logconf_range.add_variable('range.right', 'uint16_t')
        logconf_range.add_variable('range.up', 'uint16_t')
        scf.cf.log.add_config(logconf_range)
        logconf_range.data_received_cb.add_callback(log_range_callback)

        logconf_pose.start()
        logconf_twist.start()
        logconf_range.start()
        # move_rectangle(scf)
        takeoff(scf)
        logconf_pose.stop()
        logconf_twist.stop()
        logconf_range.stop()
               


