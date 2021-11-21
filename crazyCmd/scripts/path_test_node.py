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

def state_sub_cb(msg):
    global state_bag, state, state_bag_closed, isStarted
    # Saving orientation:
    state.orientation.roll = msg.orientation.roll
    state.orientation.pitch = msg.orientation.pitch
    state.orientation.yaw = msg.orientation.yaw

    # Saving linear velocity:
    vx = msg.velocity.x
    vy = msg.velocity.y
    vz = msg.velocity.z

    state.velocity.x = vx * math.cos(msg.orientation.yaw) + vy * math.sin(msg.orientation.yaw)
    state.velocity.y = - vx * math.sin(msg.orientation.yaw) + vy * math.cos(msg.orientation.yaw)
    state.velocity.z = vz
    # Saving rotating speed:
    state.rotating_speed.z = msg.rotating_speed.z

    if not state_bag_closed and isStarted:
        state_bag.write('/cf1/' + DEFAULT_CF_STATE_TOPIC, state)

def ref_cb(msg):
    global ref_state, ref_bag, state_bag_closed, state, isStarted

    vx = msg.desired_velocity.x
    vy = msg.desired_velocity.y
    vz = msg.desired_velocity.z

    ref_state.desired_velocity.x = vx * math.cos(state.orientation.yaw) + vy * math.sin(state.orientation.yaw)
    ref_state.desired_velocity.y = - vx * math.sin(state.orientation.yaw) + vy * math.cos(state.orientation.yaw)
    ref_state.desired_velocity.z = vz

    ref_state.desired_yaw_rate = msg.desired_yaw_rate

    if not state_bag_closed and isStarted:
        ref_bag.write('/cf1/' + DEFAULT_ACTUAL_DESTINATION_TOPIC, ref_state)

def motor_command_sub_cb(msg):
    global motor_bag, attitude, state_bag_closed, isStarted
    # Saving motor commands:
    attitude.desired_thrust = msg.desired_thrust
    attitude.desired_attitude.roll = msg.desired_attitude.roll
    attitude.desired_attitude.pitch = msg.desired_attitude.pitch
    attitude.desired_attitude.yaw = msg.desired_attitude.yaw

    if not state_bag_closed and isStarted:
        motor_bag.write('/cf1/' + DEFAULT_MOTOR_CMD_TOPIC, attitude)

def closing_operations():
    global state_bag, motor_bag, state_bag_closed
    if not state_bag_closed:
        state_bag.close()
        motor_bag.close()
        ref_bag.close()
        state_bag_closed = True


# INITIAL PARAMETERS
rospack = rospkg.RosPack()

pkg_path = rospack.get_path('crazyCmd')

# Rosbag init:

state_bag_closed = False

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

    # Path info:
    yaw_rate = 360.0 / 5
    velocity = 0.2
    side = 0.2

    state_bag = rosbag.Bag(pkg_path + '/data/output/Rosbags/sim_state_2.bag', 'w')
    motor_bag = rosbag.Bag(pkg_path + '/data/output/Rosbags/sim_motor_2.bag', 'w')
    ref_bag = rosbag.Bag(pkg_path + '/data/output/Rosbags/sim_ref_2.bag', 'w')
    isStarted = True
    # Wait some time:
    time.sleep(10.0)

    # Takeoff:
    takeoff_goal = TakeoffGoal()
    takeoff_goal.takeoff_height = 0.3
    takeoff_client.send_goal(takeoff_goal)
    takeoff_client.wait_for_result()
    print('TAKEOFF COMPLETE')
    time.sleep(1)

    # Side 1:
    side1_goal = Destination3DGoal()
    side1_goal.destination_info.desired_velocity.x = velocity
    side1_goal.time_duration = side / velocity
    rel_vel_client.send_goal(side1_goal)
    rel_vel_client.wait_for_result()
    print('FIRST SIDE COMPLETE')
    time.sleep(1)

    # Turn left:
    turn_goal = Destination3DGoal()
    turn_goal.destination_info.desired_yaw_rate = yaw_rate
    turn_goal.time_duration = 90.0 / yaw_rate
    rel_vel_client.send_goal(turn_goal)
    rel_vel_client.wait_for_result()
    print('TURN +90 DEG COMPLETE')
    time.sleep(1)

    # Side 2:
    side2_goal = Destination3DGoal()
    side2_goal.destination_info.desired_velocity.x = velocity
    side2_goal.time_duration = side / velocity
    rel_vel_client.send_goal(side2_goal)
    rel_vel_client.wait_for_result()
    print('SECOND SIDE COMPLETE')
    time.sleep(1)

    # Turn left:
    turn_goal = Destination3DGoal()
    turn_goal.destination_info.desired_yaw_rate = yaw_rate
    turn_goal.time_duration = 90.0 / yaw_rate
    rel_vel_client.send_goal(turn_goal)
    rel_vel_client.wait_for_result()
    print('TURN +90 DEG COMPLETE')
    time.sleep(1)

    # Side 3:
    side3_goal = Destination3DGoal()
    side3_goal.destination_info.desired_velocity.x = velocity
    side3_goal.time_duration = side / velocity
    rel_vel_client.send_goal(side3_goal)
    rel_vel_client.wait_for_result()
    print('THIRD SIDE COMPLETE')
    time.sleep(1)

    # Turn left:
    turn_goal = Destination3DGoal()
    turn_goal.destination_info.desired_yaw_rate = yaw_rate
    turn_goal.time_duration = 90.0 / yaw_rate
    rel_vel_client.send_goal(turn_goal)
    rel_vel_client.wait_for_result()
    print('TURN +90 DEG COMPLETE')
    time.sleep(1)

    # Side 4:
    side4_goal = Destination3DGoal()
    side4_goal.destination_info.desired_velocity.x = velocity
    side4_goal.time_duration = side / velocity
    rel_vel_client.send_goal(side4_goal)
    rel_vel_client.wait_for_result()
    print('FOURTH SIDE COMPLETE')
    time.sleep(1)

    # Land:
    landing_goal = TakeoffGoal()
    landing_goal.takeoff_height = 0.1
    land_client.send_goal(landing_goal)
    land_client.wait_for_result()
    print('LANDING COMPLETE')
    time.sleep(2)

    state_bag_closed = True
    state_bag.close()
    motor_bag.close()
    ref_bag.close()


    rospy.spin()