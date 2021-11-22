#! /usr/bin/env python3
# ROS MODULES
import time
import math
import rospy
import rosbag
import rospkg
import actionlib

from crazy_common_py.default_topics import DEFAULT_CF_STATE_TOPIC, DEFAULT_TAKEOFF_ACT_TOPIC, DEFAULT_LAND_ACT_TOPIC, \
    DEFAULT_REL_POS_TOPIC, DEFAULT_REL_VEL_TOPIC, DEFAULT_MOTOR_CMD_TOPIC, DEFAULT_DESIRED_MOTOR_CMD_TOPIC, \
    DEFAULT_ACTUAL_DESTINATION_TOPIC
from crazy_common_py.dataTypes import Vector3
from crazyflie_simulator.CrazySim import CrazySim
from crazyflie_messages.msg import CrazyflieState, Attitude, Position
from crazyflie_messages.msg import TakeoffAction, TakeoffGoal
from crazyflie_messages.msg import Destination3DAction, Destination3DGoal
from enum import Enum


# Functions:
state = CrazyflieState()
attitude = Attitude()
ref_state = Position()

isStarted = False
bags_closed = False
# ======================================================================================================================
#                                             INITIAL PARAMETERS
# ----------------------------------------------------------------------------------------------------------------------
# Real (True) or simulated (False) crazyflie:
isReal = False


# Path info:
yaw_rate = 360.0 / 5
turning_angle = 90.0
side1 = 0.2
side2 = 0.2
side3 = 0.2
side4 = 0.2
velocity1 = 0.2
velocity2 = 0.2
velocity3 = 0.2
velocity4 = 0.2

# Rosbag name:
'''
    Rectangular: R_D_N1_N2_N3_N4_V_N5_N6_N7_N8_RA_N9_RR_N10
        - N1, N2, N3, N4 -> length of sides [m]
        - N5, N6, N7, N8 -> velocity on each side [m/s]
        - N9 -> angle during turning [deg]
        - N10 -> rotating speed for turning [deg/s]
        
        'R_D_04_02_04_02_V_02_02_02_02_RA_90_RR_72_N1'
        
        'S_D_02_02_02_02_V_02_02_02_02_RA_90_RR_72_N1'
'''
experiment_name = 'S_D_02_02_02_02_V_02_02_02_02_RA_90_RR_72_N1'
# ======================================================================================================================



def state_sub_cb(msg):
    global state_bag, state, bags_closed, isStarted, isReal
    # Saving orientation:
    state.orientation.roll = msg.orientation.roll
    state.orientation.pitch = msg.orientation.pitch
    state.orientation.yaw = msg.orientation.yaw

    # Saving position:
    state.position.x = msg.position.x
    state.position.y = msg.position.y
    state.position.z = msg.position.z

    # Saving velocity:
    if isReal:
        state.velocity.x = msg.velocity.x
        state.velocity.y = msg.velocity.y
        state.velocity.z = msg.velocity.z
    else:
        vx = msg.velocity.x
        vy = msg.velocity.y
        vz = msg.velocity.z

        '''state.velocity.x = vx * math.cos(msg.orientation.yaw) + vy * math.sin(msg.orientation.yaw)
        state.velocity.y = - vx * math.sin(msg.orientation.yaw) + vy * math.cos(msg.orientation.yaw)
        state.velocity.z = vz'''
        state.velocity.x = vx
        state.velocity.y = vy
        state.velocity.z = vz

    # Saving rotating speed:
    state.rotating_speed.x = msg.rotating_speed.x
    state.rotating_speed.y = msg.rotating_speed.y
    state.rotating_speed.z = msg.rotating_speed.z

    if (not bags_closed) and isStarted:
        state_bag.write('/cf1/' + DEFAULT_CF_STATE_TOPIC, state)

def ref_cb(msg):
    global ref_state, ref_bag, bags_closed, state, isStarted, isReal

    # Saving reference velocity:
    if isReal:
        ref_state.desired_velocity.x = msg.desired_velocity.x
        ref_state.desired_velocity.y = msg.desired_velocity.y
        ref_state.desired_velocity.z = msg.desired_velocity.z
    else:
        vx = msg.desired_velocity.x
        vy = msg.desired_velocity.y
        vz = msg.desired_velocity.z

        ref_state.desired_velocity.x = vx * math.cos(state.orientation.yaw) + vy * math.sin(state.orientation.yaw)
        ref_state.desired_velocity.y = - vx * math.sin(state.orientation.yaw) + vy * math.cos(state.orientation.yaw)
        ref_state.desired_velocity.z = vz

    # Saving reference yaw rate:
    ref_state.desired_yaw_rate = msg.desired_yaw_rate

    if (not bags_closed) and isStarted:
        ref_bag.write('/cf1/' + DEFAULT_ACTUAL_DESTINATION_TOPIC, ref_state)

def motor_command_sub_cb(msg):
    global motor_bag, attitude, bags_closed, isStarted
    # Saving motor commands:
    attitude.desired_thrust = msg.desired_thrust
    attitude.desired_attitude.roll = msg.desired_attitude.roll
    attitude.desired_attitude.pitch = msg.desired_attitude.pitch
    attitude.desired_attitude.yaw = msg.desired_attitude.yaw

    if (not bags_closed) and isStarted:
        motor_bag.write('/cf1/' + DEFAULT_MOTOR_CMD_TOPIC, attitude)

def closing_operations():
    global state_bag, motor_bag, bags_closed
    if not bags_closed:
        state_bag.close()
        motor_bag.close()
        ref_bag.close()
        bags_closed = True


# INITIAL PARAMETERS
rospack = rospkg.RosPack()

pkg_path = rospack.get_path('crazyCmd')

# Rosbag init:



if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('path_test_node', log_level=rospy.INFO)

    # Subscribers:
    state_sub = rospy.Subscriber('/cf1/' + DEFAULT_CF_STATE_TOPIC, CrazyflieState, state_sub_cb)
    motor_command_sub = rospy.Subscriber('/cf1/' + DEFAULT_MOTOR_CMD_TOPIC, Attitude, motor_command_sub_cb)
    ref_state_sub = rospy.Subscriber('/cf1/' + DEFAULT_ACTUAL_DESTINATION_TOPIC, Position, ref_cb)

    # ACTION CLIENTs
    # Takeoff client:
    takeoff_client = actionlib.SimpleActionClient('/cf1/' + DEFAULT_TAKEOFF_ACT_TOPIC, TakeoffAction)
    takeoff_client.wait_for_server()

    # Land client:
    land_client = actionlib.SimpleActionClient('/cf1/' + DEFAULT_LAND_ACT_TOPIC, TakeoffAction)
    land_client.wait_for_server()

    # Relative displacement:
    rel_disp_client = actionlib.SimpleActionClient('/cf1/' + DEFAULT_REL_POS_TOPIC, Destination3DAction)
    rel_disp_client.wait_for_server()

    # Relative velocity:
    rel_vel_client = actionlib.SimpleActionClient('/cf1/' + DEFAULT_REL_VEL_TOPIC, Destination3DAction)
    rel_vel_client.wait_for_server()

    rospy.on_shutdown(closing_operations)

    if isReal:
        state_bag_name = 'real_state_' + experiment_name + '.bag'
        motor_bag_name = 'real_motor_' + experiment_name + '.bag'
        ref_bag_name = 'real_ref_' + experiment_name + '.bag'
    else:
        state_bag_name = 'sim_state_' + experiment_name + '.bag'
        motor_bag_name = 'sim_motor_' + experiment_name + '.bag'
        ref_bag_name = 'sim_ref_' + experiment_name + '.bag'

    state_bag = rosbag.Bag(pkg_path + '/data/output/Rosbags/' + state_bag_name, 'w')
    motor_bag = rosbag.Bag(pkg_path + '/data/output/Rosbags/' + motor_bag_name, 'w')
    ref_bag = rosbag.Bag(pkg_path + '/data/output/Rosbags/' + ref_bag_name, 'w')

    # Wait some time:
    rospy.sleep(10.0)
    isStarted = True

    # Takeoff:
    takeoff_goal = TakeoffGoal()
    takeoff_goal.takeoff_height = 0.3
    takeoff_client.send_goal(takeoff_goal)
    takeoff_client.wait_for_result()
    print('TAKEOFF COMPLETE')
    rospy.sleep(1.0)

    # Side 1:
    side1_goal = Destination3DGoal()
    side1_goal.destination_info.desired_velocity.x = velocity1
    side1_goal.time_duration = side1 / velocity1
    rel_vel_client.send_goal(side1_goal)
    rel_vel_client.wait_for_result()
    print('FIRST SIDE COMPLETE')
    rospy.sleep(1.0)

    # Turn left:
    turn_goal = Destination3DGoal()
    turn_goal.destination_info.desired_yaw_rate = yaw_rate
    turn_goal.time_duration = turning_angle / yaw_rate
    rel_vel_client.send_goal(turn_goal)
    rel_vel_client.wait_for_result()
    print('TURN +90 DEG COMPLETE')
    rospy.sleep(1.0)

    # Side 2:
    side2_goal = Destination3DGoal()
    side2_goal.destination_info.desired_velocity.x = velocity2
    side2_goal.time_duration = side2 / velocity2
    rel_vel_client.send_goal(side2_goal)
    rel_vel_client.wait_for_result()
    print('SECOND SIDE COMPLETE')
    rospy.sleep(1.0)

    # Turn left:
    turn_goal = Destination3DGoal()
    turn_goal.destination_info.desired_yaw_rate = yaw_rate
    turn_goal.time_duration = turning_angle / yaw_rate
    rel_vel_client.send_goal(turn_goal)
    rel_vel_client.wait_for_result()
    print('TURN +90 DEG COMPLETE')
    rospy.sleep(1.0)

    # Side 3:
    side3_goal = Destination3DGoal()
    side3_goal.destination_info.desired_velocity.x = velocity3
    side3_goal.time_duration = side3 / velocity3
    rel_vel_client.send_goal(side3_goal)
    rel_vel_client.wait_for_result()
    print('THIRD SIDE COMPLETE')
    rospy.sleep(1.0)

    # Turn left:
    turn_goal = Destination3DGoal()
    turn_goal.destination_info.desired_yaw_rate = yaw_rate
    turn_goal.time_duration = turning_angle / yaw_rate
    rel_vel_client.send_goal(turn_goal)
    rel_vel_client.wait_for_result()
    print('TURN +90 DEG COMPLETE')
    rospy.sleep(1.0)

    # Side 4:
    side4_goal = Destination3DGoal()
    side4_goal.destination_info.desired_velocity.x = velocity4
    side4_goal.time_duration = side4 / velocity4
    rel_vel_client.send_goal(side4_goal)
    rel_vel_client.wait_for_result()
    print('FOURTH SIDE COMPLETE')
    rospy.sleep(5.0)

    # Land:
    landing_goal = TakeoffGoal()
    landing_goal.takeoff_height = 0.1
    land_client.send_goal(landing_goal)
    land_client.wait_for_result()
    print('LANDING COMPLETE')
    rospy.sleep(5.0)

    bags_closed = True
    state_bag.close()
    motor_bag.close()
    ref_bag.close()

    print('\n\nTASK COMPLETE\n\n')
    rospy.spin()