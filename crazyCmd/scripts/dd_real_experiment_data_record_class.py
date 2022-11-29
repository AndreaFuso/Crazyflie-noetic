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


class DataCollection():
    def __init__(self):
        # URI to the Crazyflie to connect to
        self.uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E2')

        # Default parameters
        self.DEFAULT_HEIGHT = 0.5

        # Only output errors from the logging framework
        logging.basicConfig(level=logging.ERROR)

        # Initialize the low-level drivers
        cflib.crtp.init_drivers()

        # Configure the Logging
        self.logconf_pose = LogConfig(name='StatePose', period_in_ms=10)
        self.logconf_pose.add_variable('stateEstimate.x', 'float') #[m]
        self.logconf_pose.add_variable('stateEstimate.y', 'float')
        self.logconf_pose.add_variable('stateEstimate.z', 'float')
        self.logconf_pose.add_variable('stateEstimate.roll', 'float') #[degree]
        self.logconf_pose.add_variable('stateEstimate.pitch', 'float')
        self.logconf_pose.add_variable('stateEstimate.yaw', 'float')
        
        self.logconf_twist = LogConfig(name='StateTwist', period_in_ms=10)
        self.logconf_twist.add_variable('stateEstimate.vx', 'float') #[m/s]
        self.logconf_twist.add_variable('stateEstimate.vy', 'float')
        self.logconf_twist.add_variable('stateEstimate.vz', 'float')
        self.logconf_twist.add_variable('stateEstimateZ.rateRoll', 'int16_t') # [milliradians/s]
        self.logconf_twist.add_variable('stateEstimateZ.ratePitch', 'int16_t')
        self.logconf_twist.add_variable('stateEstimateZ.rateYaw', 'int16_t')
        
        # Publisher
        self.pose_pub = rospy.Publisher("/state_pose_real",CrazyflieState , queue_size=1)
        self.twist_pub = rospy.Publisher("/state_twist_real",CrazyflieState , queue_size=1)

    def simple_connect(self):
        print("Yeah, I'm connected! :D")
        # time.sleep(3)
        # print("Now I will disconnect :'(")    

######## Asynchronous state logging ########
    # for the pose
    def log_state_pose_callback(self, timestamp, data, logconf_state_pose):
        print('[%d][%s]: %s' % (timestamp, logconf_state_pose.name, data))
        pose_msg = CrazyflieState()
        pose_msg.name = 'cf1'
        pose_msg.position.x = data['stateEstimate.x']
        pose_msg.position.y = data['stateEstimate.y']
        pose_msg.position.z = data['stateEstimate.z']
        pose_msg.orientation.roll = data['stateEstimate.roll']
        pose_msg.orientation.pitch = data['stateEstimate.pitch']
        pose_msg.orientation.yaw = data['stateEstimate.yaw']
        self.pose_pub.publish(pose_msg)

    def state_pose_log_async(self,scf,logconf):
        cf = scf.cf
        cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(self.log_state_pose_callback)
        logconf.start()
        time.sleep(120)
        logconf.stop()

    # for the twist
    def log_state_twist_callback(self, timestamp, data, logconf_state_twist):
        print('[%d][%s]: %s' % (timestamp, logconf_state_twist.name, data))
        twist_msg = CrazyflieState()
        twist_msg.name = 'cf1'
        twist_msg.velocity.x = data['stateEstimate.vx']
        twist_msg.velocity.y = data['stateEstimate.vy']
        twist_msg.velocity.z = data['stateEstimate.vz']
        twist_msg.rotating_speed.x = data['stateEstimateZ.rateRoll']/1000*180/math.pi #[degree/s]
        twist_msg.rotating_speed.y = data['stateEstimateZ.ratePitch']/1000*180/math.pi
        twist_msg.rotating_speed.z = data['stateEstimateZ.rateYaw']/1000*180/math.pi
        self.twist_pub.publish(twist_msg)

    def state_twist_log_async(self, scf,logconf):
        cf = scf.cf
        cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(self.log_state_twist_callback)
        logconf.start()
        time.sleep(120)
        logconf.stop()

    def take_action(self):
        with SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            ######## Asynchronous state logging ########
            self.state_pose_log_async(scf,self.logconf_pose)
            self.state_twist_log_async(scf,self.logconf_twist)

if __name__ == '__main__': 
    rospy.init_node('optic_flow_calculator', anonymous=True)
    data_collection= DataCollection()
    data_collection.take_action()      



