#! /usr/bin/env python3
# ROS MODULES
import time
import math
import rospy
import rosbag
import rospkg
import actionlib
from enum import Enum
from crazy_common_py.default_topics import DEFAULT_CF_STATE_TOPIC, DEFAULT_REL_POS_TOPIC, DEFAULT_TAKEOFF_ACT_TOPIC, \
    DEFAULT_FLOCK_TOPIC, DEFAULT_REL_VEL_TOPIC, DEFAULT_100Hz_PACE_TOPIC
from crazyflie_messages.msg import CrazyflieState, SwarmStates
from crazyflie_messages.msg import Destination3DAction, Destination3DGoal, Destination3DResult, Destination3DFeedback
from crazyflie_messages.msg import TakeoffAction, TakeoffGoal, TakeoffResult, TakeoffFeedback
from crazyflie_messages.msg import EmptyAction, EmptyGoal, EmptyResult
from std_msgs.msg import Empty
from crazy_common_py.common_functions import deg2rad, rad2deg
from rosgraph_msgs.msg import Clock


clock = Clock()
states = SwarmStates()
canWriteBags = False
experiment_name = 'flocking_N3'

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('crazyCmd')
# ======================================================================================================================
#                                           C A L L B A C K  F U N C T I O N S
# ======================================================================================================================
# States subscriber callback function:
def states_sub_cb(msg):
    global states
    states = msg

# Closing operations:
def closing_operations():
    global canWriteBags, states_bag, clock_bag
    canWriteBags = False
    states_bag.close()
    clock_bag.close()
    print('\n\nEXITING, ALL BAGS CLOSED\n\n')

def clock_sub_cb(msg):
    global clock
    clock = msg

def pace100Hz_sub_cb(msg):
    global states, canWriteBags, clock, states_bag, clock_bag
    actual_states = states
    actual_clock = clock

    if canWriteBags:
        states_bag.write('/swarm/states', actual_states)
        clock_bag.write('/clock', actual_clock)

if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('flocking_test_node', log_level=rospy.ERROR)

    rospy.on_shutdown(closing_operations)

    # Subscribers:
    states_sub = rospy.Subscriber('/swarm/states', SwarmStates, states_sub_cb)
    clock_sub = rospy.Subscriber('/clock', Clock, clock_sub_cb)
    pace_100Hz_sub = rospy.Subscriber(DEFAULT_100Hz_PACE_TOPIC, Empty, pace100Hz_sub_cb)

    # Action clients:
    rel_pos_action_client = actionlib.SimpleActionClient('/cf1/' + DEFAULT_REL_POS_TOPIC, Destination3DAction)
    rel_pos_action_client.wait_for_server()

    rel_vel_action_client = actionlib.SimpleActionClient('/cf1/' + DEFAULT_REL_VEL_TOPIC, Destination3DAction)
    rel_vel_action_client.wait_for_server()

    takeoff_action_client = actionlib.SimpleActionClient('/swarm/takeoff_actn', TakeoffAction)
    takeoff_action_client.wait_for_server()

    flocking_action_client = actionlib.SimpleActionClient('/swarm/flocking_actn', EmptyAction)
    flocking_action_client.wait_for_server()

    states_bag_name = 'sim_states_' + experiment_name + '.bag'
    clock_bag_name = 'sim_clock_' + experiment_name + '.bag'

    states_bag = rosbag.Bag(pkg_path + '/data/output/Rosbags/' + states_bag_name, 'w')
    clock_bag = rosbag.Bag(pkg_path + '/data/output/Rosbags/' + clock_bag_name, 'w')

    rospy.sleep(1)

    # Takeoff:
    print('\n\nSTARTING TAKEOFF\n\n')
    takeoff_request = TakeoffGoal()
    takeoff_request.takeoff_height = 2.0
    takeoff_action_client.send_goal(takeoff_request)
    takeoff_action_client.wait_for_result()
    print('\n\nTAKEOFF COMPLETED\n\n')
    rospy.sleep(10)

    # Moving cf1 in position:
    print('\n\nSTARTING MOTION OF CF1\n\n')
    cf1_desired_destination = Destination3DGoal()
    cf1_desired_destination.destination_info.desired_position.x = 2.0
    cf1_desired_destination.destination_info.desired_position.y = 0.0
    cf1_desired_destination.destination_info.desired_position.z = 0.5
    rel_pos_action_client.send_goal(cf1_desired_destination)
    rel_pos_action_client.wait_for_result()
    print('\n\nMOTION OF CF1 COMPLETED\n\n')
    rospy.sleep(3)

    # Start flocking:
    print('\n\nSTARTING FLOCKING\n\n')
    canWriteBags = True
    flocking_request = EmptyGoal()
    flocking_action_client.send_goal(flocking_request)
    flocking_action_client.wait_for_result()

    # Start circular moption of cf1:
    print('\n\nSTARTING CF1 CIRCULAR MOTION\n\n')
    cf1_circular_path_request = Destination3DGoal()
    radius = 2.0
    rotating_speed = 20
    des_vx = deg2rad(rotating_speed) * radius
    cf1_circular_path_request.destination_info.desired_velocity.x = des_vx
    cf1_circular_path_request.destination_info.desired_yaw_rate = rotating_speed
    rel_vel_action_client.send_goal(cf1_circular_path_request)
    rel_vel_action_client.wait_for_result()

    rospy.spin()