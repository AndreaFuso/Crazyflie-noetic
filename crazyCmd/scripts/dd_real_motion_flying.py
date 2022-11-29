#! /usr/bin/env python3

import logging
import sys
import time
from threading import Event
import rospy

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# a wrapper around the position setpoint frame work of the crazyflie
from cflib.positioning.motion_commander import MotionCommander

from cflib.utils import uri_helper


URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E2')

DEFAULT_HEIGHT = 0.5
BOX_LIMIT = 0.3

# Global variable deck_attached_event
deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

# Global variables estimated position
position_estimate = [0, 0]

def log_pos_callback(timestamp, data, logconf):
    print(data)

# To check if the deck is attached or not
def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')

# The takeoff function to make it fly for 3 secs
def take_off_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        # mc.up(0.2) # change the height: increase 0.2m based on default height using mc.up(value); or change DEFAULT_HEIGHT value
        time.sleep(3)
        mc.stop()

# logging the positions
def log_pos_callback(timestamp, data, logconf):
    print(data)
    #### to manage the data, we can replace the print function with a plotter, using python lib matplotlib
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']

def move_box_limit(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        mc.start_forward()

        ### Back and forth with limits
        # while (1):
        #     if position_estimate[0] > BOX_LIMIT:
        #         mc.start_back()
        #     elif position_estimate[0] < -BOX_LIMIT:
        #         mc.start_forward()
        #     time.sleep(0.1)

        ### Bouncing in a bounding box
        body_x_cmd = 0.2
        body_y_cmd = 0.1
        max_vel = 0.2

        while (1):
            if position_estimate[0] > BOX_LIMIT:
                 body_x_cmd=-max_vel
            elif position_estimate[0] < -BOX_LIMIT:
                body_x_cmd=max_vel
            if position_estimate[1] > BOX_LIMIT:
                body_y_cmd=-max_vel
            elif position_estimate[1] < -BOX_LIMIT:
                body_y_cmd=max_vel

            mc.start_linear_motion(body_x_cmd, body_y_cmd, 0)

            time.sleep(0.1)



# To move in a direction!
def move_linear_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(1)
        mc.forward(0.5)
        time.sleep(1)
        mc.turn_left(180)
        time.sleep(1)
        mc.forward(0.5)
        time.sleep(1)

if __name__ == '__main__':

    rospy.init_node('move_box_limit')

    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)
        
        # These two are to check if the deck is correctly attached before flying
        scf.cf.param.add_update_callback(group="deck", name="bcFlow2", cb=param_deck_flow)
        time.sleep(1)

        # We are now using deck_attached_event.wait()? If this returns false, 
        # the function will not be called and the crazyflie will not take off.
        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        # take_off_simple(scf)
        logconf.start()
        move_linear_simple(scf)
        # move_box_limit(scf)
        logconf.stop()

    # rospy.spin()





