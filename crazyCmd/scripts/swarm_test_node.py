#! /usr/bin/env python3
# ROS MODULES
import time
import math
import rospy
import rosbag
import rospkg
import actionlib
from enum import Enum
from crazy_common_py.default_topics import DEFAULT_CF_STATE_TOPIC, DEFAULT_REL_VEL_TOPIC, DEFAULT_TAKEOFF_ACT_TOPIC
from crazyflie_messages.msg import CrazyflieState, SwarmStates
from crazyflie_messages.msg import Destination3DAction, Destination3DGoal, Destination3DResult, Destination3DFeedback
from crazyflie_messages.msg import TakeoffAction, TakeoffGoal, TakeoffResult, TakeoffFeedback


class TaskType(Enum):
    LINEAR_AND_ROUNDS = 0
    ROUNDS = 1
    AROUND_AXIS = 2


canWriteBag = False
# ======================================================================================================================
#                                          S I M U L A T I O N  P A R A M E T E R S
# ======================================================================================================================
task_type = TaskType.LINEAR_AND_ROUNDS
takeoff_height = 0.5        # [m]
saveResult = False
# ----------------------------------------------------------------------------------------------------------------------
#                                       L I N E A R  A N D  R O U N D S
#
# The pyramid goes forwards and then starts rotating around its axis completing a certain amount of rotations.
# ----------------------------------------------------------------------------------------------------------------------
# Forward displacement along x:
LR_forward_displacement_distance = 2.0          # [m]
LR_forward_displacement_velocity = 0.5          # [m/s]

# Rotation:
LR_rotating_speed = 45.0       # [deg/s]
LR_number_of_rotations = 1


# Bag name:
bag_file_name_LR = 'PS_LR_N2.bag'

# ----------------------------------------------------------------------------------------------------------------------
#                                                      R O U N D S
#
# The pyramid starts rotating around its axis completing a certain amount of rotations.
# ----------------------------------------------------------------------------------------------------------------------
# Rotation:
R_rotating_speed = 45           # [deg/s]
R_number_or_rotations = 1

# Bag name:
bag_file_name_R = 'PS_R_N1.bag'
# ----------------------------------------------------------------------------------------------------------------------
#                                                 A R O U N D  A X I S
#
# The pyramid goes forward, turns 90 deg and then starts rotating around origin.
# ----------------------------------------------------------------------------------------------------------------------
# Forward path:
AA_forward_displacement_distance = 3.0      # [m]
AA_forward_displacement_velocity = 0.5      # [m/s]

# 90 deg turn:
AA_turn_deg = 90.0      # [deg]
AA_turn_rotating_speed = 45  # [deg/s]

# Rotation around origin:
AA_axis_rotating_speed = 10     # [deg/s]
AA_axis_number_of_rotations = 1

# Bag name:
bag_file_name_AA = 'PS_AA_N1.bag'
# ======================================================================================================================
#                                           C A L L B A C K  F U N C T I O N S
# ======================================================================================================================
# States subscriber callback function:
def states_sub_cb(msg):
    global canWriteBag, states_bag, saveResult
    if canWriteBag and saveResult:
        states_bag.write('/pyramid_swarm/states', msg)


# Closing operations:
def closing_operations():
    global canWriteBag, states_bag, saveResult
    canWriteBag = False
    if saveResult:
        states_bag.close()
# ======================================================================================================================
#                                             N O D E  M A I N  R O U T I N E
# ======================================================================================================================
if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('swarm_test_node', log_level=rospy.ERROR)

    # Operations to be performed if shutdown occurs:
    rospy.on_shutdown(closing_operations)

    # Subscribers:
    states_sub = rospy.Subscriber('/pyramid_swarm/states', SwarmStates, states_sub_cb)

    # Action clients:
    rel_vel_action_client = actionlib.SimpleActionClient('/pyramid_swarm/' + DEFAULT_REL_VEL_TOPIC, Destination3DAction)
    rel_vel_action_client.wait_for_server()
    takeoff_action_client = actionlib.SimpleActionClient('/pyramid_swarm/' + DEFAULT_TAKEOFF_ACT_TOPIC, TakeoffAction)
    takeoff_action_client.wait_for_server()

    # Path to package:
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('crazyCmd')

    if saveResult:
        if task_type == TaskType.LINEAR_AND_ROUNDS:
            # Path to rosbag:
            bag_path = pkg_path + '/data/output/Rosbags/' + bag_file_name_LR
        elif task_type == TaskType.ROUNDS:
            # Path to rosbag:
            bag_path = pkg_path + '/data/output/Rosbags/' + bag_file_name_R
        else:
            # Path to rosbag:
            bag_path = pkg_path + '/data/output/Rosbags/' + bag_file_name_AA

        # Bag instance:
        states_bag = rosbag.Bag(bag_path, 'w')


    rospy.sleep(10.0)

    if task_type == TaskType.LINEAR_AND_ROUNDS:
        # Takeoff:
        print('Taking off...')
        takeoff_goal = TakeoffGoal()
        takeoff_goal.takeoff_height = takeoff_height

        takeoff_action_client.send_goal(takeoff_goal)
        takeoff_action_client.wait_for_result()
        print('Takeoff COMPLETED\n')
        rospy.sleep(5.0)
        canWriteBag = True

        # Forward motion:
        print('Going forward...')
        forward_goal = Destination3DGoal()
        forward_goal.destination_info.desired_velocity.x = LR_forward_displacement_velocity
        forward_goal.time_duration = LR_forward_displacement_distance / LR_forward_displacement_velocity

        rel_vel_action_client.send_goal(forward_goal)
        rel_vel_action_client.wait_for_result()
        print('Forward displacement COMPLETED\n')
        rospy.sleep(5.0)

        # Axial rotation:
        rotation_goal = Destination3DGoal()
        rotation_goal.destination_info.desired_yaw_rate = LR_rotating_speed
        rotation_goal.time_duration = (LR_number_of_rotations * 360) / LR_rotating_speed

        rel_vel_action_client.send_goal(rotation_goal)
        rel_vel_action_client.wait_for_result()

    elif task_type == TaskType.ROUNDS:
        # Takeoff:
        print('Taking off...')
        takeoff_goal = TakeoffGoal()
        takeoff_goal.takeoff_height = takeoff_height

        takeoff_action_client.send_goal(takeoff_goal)
        takeoff_action_client.wait_for_result()
        print('Takeoff COMPLETED\n')
        rospy.sleep(5.0)

        # Axial rotation:
        print('Rotating around pyramid axis...')
        rotation_goal = Destination3DGoal()
        rotation_goal.destination_info.desired_yaw_rate = R_rotating_speed
        rotation_goal.time_duration = (R_number_or_rotations * 360) / R_rotating_speed

        rel_vel_action_client.send_goal(rotation_goal)
        rel_vel_action_client.wait_for_result()
        print('Rotation around pyramid axis COMPLETED\n')

    elif task_type == TaskType.AROUND_AXIS:
        # Takeoff:
        print('Taking off...')
        takeoff_goal = TakeoffGoal()
        takeoff_goal.takeoff_height = takeoff_height

        takeoff_action_client.send_goal(takeoff_goal)
        takeoff_action_client.wait_for_result()
        print('Takeoff COMPLETED\n')
        rospy.sleep(1.0)

        # Forward motion:
        print('Going forward...')
        forward_goal = Destination3DGoal()
        forward_goal.destination_info.desired_velocity.x = AA_forward_displacement_velocity
        forward_goal.time_duration = AA_forward_displacement_distance / AA_forward_displacement_velocity

        rel_vel_action_client.send_goal(forward_goal)
        rel_vel_action_client.wait_for_result()
        print('Forward displacement COMPLETED\n')
        rospy.sleep(2.0)

        # 90 deg rotation:
        print('Rotating 90 deg...')
        rotation_goal = Destination3DGoal()
        rotation_goal.destination_info.desired_yaw_rate = AA_turn_rotating_speed
        rotation_goal.time_duration = 90.0 / AA_turn_rotating_speed

        rel_vel_action_client.send_goal(rotation_goal)
        rel_vel_action_client.wait_for_result()
        print('Rotation 90 deg COMPLETED\n')
        time.sleep(5.0)

        # Rotation around origin:
        print('Rotating around origin...')
        origin_rotation_goal = Destination3DGoal()
        origin_rotation_goal.destination_info.desired_yaw = 1
        origin_rotation_goal.destination_info.desired_yaw_rate = AA_axis_rotating_speed
        origin_rotation_goal.time_duration = (AA_axis_number_of_rotations * 360) / AA_axis_rotating_speed

        rel_vel_action_client.send_goal(origin_rotation_goal)
        rel_vel_action_client.wait_for_result()
        print('Rotation around origin COMPLETED\n')

    canWriteBag = False
    if saveResult:
        states_bag.close()
        rospy.spin()