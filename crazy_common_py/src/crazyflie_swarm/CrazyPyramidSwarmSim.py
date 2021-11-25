# ROS MODULES
import time

import rospy
import actionlib

import math
from math import atan2

# Action
from crazyflie_messages.msg import TakeoffAction, TakeoffGoal, TakeoffResult, TakeoffFeedback
from crazyflie_messages.msg import Destination3DAction, Destination3DGoal, Destination3DResult, Destination3DFeedback
from crazyflie_messages.msg import VelocityTrajectoryAction, VelocityTrajectoryGoal, VelocityTrajectoryResult, VelocityTrajectoryFeedback
from crazyflie_messages.msg import EmptyAction, EmptyGoal, EmptyResult, EmptyFeedback

from std_msgs.msg import Empty
from crazyflie_messages.msg import Position, CrazyflieState, SwarmStates
from crazy_common_py.dataTypes import Vector3
from crazy_common_py.common_functions import deg2rad, rad2deg, extractCfNumber

from crazy_common_py.default_topics import DEFAULT_FLOCK_TOPIC, DEFAULT_CF_STATE_TOPIC, DEFAULT_100Hz_PACE_TOPIC, \
    DEFAULT_STOP_TOPIC, DEFAULT_REL_VEL_TOPIC
from crazy_common_py.constants import DEFAULT_LEADER, MAX_VELOCITY_X, MAX_VELOCITY_Y

class CrazyPyramidSwarmSim:
    # ==================================================================================================================
    #
    #                                               C O N S T R U C T O R
    #
    # This class completely handle one virtual pyramid swarm of crazyflies.
    # INPUTS:
    #   1) cf_names -> list of names of each Crazyflie within the virtual swarm;
    #
    # ==================================================================================================================
    def __init__(self, cf_names, number_of_levels, vertical_offset):
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                           P R O P E R T I E S  I N I T I A L I Z A T I O N
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # List of crazyflies names:
        self.cf_names = cf_names

        # Number of crazyflies:
        self.number_of_cfs = len(self.cf_names)

        # Pyramid parameters:
        self.levels = number_of_levels
        self.vertical_offset = vertical_offset

        # List of clients for takeoff action per each drone:
        self.takeoff_act_clients = []
        self.__make_takeoff_clients()

        # List of clients for flocking action:
        self.flocking_act_clients = []
        self.__make_flocking_clients()

        # List of clients for stop action:
        self.stop_clients = []
        self.__make_stop_clients()

        # List of relative velocity motion clients:
        self.rel_vel_clients = []
        self.__make_rel_vel_clients()

        # List of state subscribers:
        self.state_subs = []
        self.states = []
        self.__make_state_subs()

        # List of clients for relative destination action per each drone:
        #self.relative_motion_act_clients = []
        #self.__make_relative_motion_clients()

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       S U B S C R I B E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Subscriber to pace 100Hz:
        self.pace_100Hz_sub = rospy.Subscriber('/' + DEFAULT_100Hz_PACE_TOPIC, Empty, self.__pace_100Hz_sub_callback)
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       P U B L I S H E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # States publisher:
        self.states_pub = rospy.Publisher('/pyramid_swarm/states', SwarmStates, queue_size=1)

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                           S E R V I C E S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                           A C T I O N S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Action to take off the entire swarm:
        self.__swarm_takeoff_act = actionlib.SimpleActionServer('/pyramid_swarm/takeoff_actn', TakeoffAction,
                                                                self.__swarm_takeoff_act_callback, False)
        self.__swarm_takeoff_act.start()

        # Action to perform a relative motion of the pyramid:
        self.__swarm_flocking_act = actionlib.SimpleActionServer('/pyramid_swarm/flocking_actn', EmptyAction,
                                                                 self.__swarm_flocking_act_callback, False)
        self.__swarm_flocking_act.start()

        # Stop action:
        self.__swarm_stop_act = actionlib.SimpleActionServer('/pyramid_swarm/' + DEFAULT_STOP_TOPIC, EmptyAction,
                                                             self.__swarm_stop_act_callback, False)
        self.__swarm_stop_act.start()

        # Relative velocity action:
        self.__swarm_rel_vel_act = actionlib.SimpleActionServer('/pyramid_swarm/' + DEFAULT_REL_VEL_TOPIC,
                                                                Destination3DAction,
                                                                self.__swarm_rel_vel_act_callback, False)
        self.__swarm_rel_vel_act.start()

    # ==================================================================================================================
    #
    #                                   I N I T I A L  O P E R A T I O N S  M E T H O D S
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                   __M A K E _ T A K E O F F _ C L I E N T S
    #
    # This method is used to create one action client per each drone, in order to perform takeoff action.
    # ------------------------------------------------------------------------------------------------------------------
    def __make_takeoff_clients(self):
        for cf_name in self.cf_names:
            tmp_action = actionlib.SimpleActionClient('/' + cf_name + '/takeoff_actn', TakeoffAction)
            self.takeoff_act_clients.append(tmp_action)
            self.takeoff_act_clients[-1].wait_for_server()

    def __make_flocking_clients(self):
        for cf_name in self.cf_names:
            if cf_name != DEFAULT_LEADER:
                tmp_action = actionlib.SimpleActionClient('/' + cf_name + '/' + DEFAULT_FLOCK_TOPIC, EmptyAction)
                self.flocking_act_clients.append(tmp_action)
                self.flocking_act_clients[-1].wait_for_server()

    '''def __make_relative_motion_clients(self):
        for cf_name in self.cf_names:
            tmp_action = actionlib.SimpleActionClient('/' + cf_name + '/relative_position_3D_motion', Destination3DAction)
            self.relative_motion_act_clients.append(tmp_action)
            self.relative_motion_act_clients[-1].wait_for_server()'''

    def __make_stop_clients(self):
        for cf_name in self.cf_names:
            tmp_action = actionlib.SimpleActionClient('/' + cf_name + '/' + DEFAULT_STOP_TOPIC, EmptyAction)
            self.stop_clients.append(tmp_action)
            self.stop_clients[-1].wait_for_server()

    def __make_rel_vel_clients(self):
        for cf_name in self.cf_names:
            tmp_action = actionlib.SimpleActionClient('/' + cf_name + '/' + DEFAULT_REL_VEL_TOPIC, Destination3DAction)
            self.rel_vel_clients.append(tmp_action)
            self.rel_vel_clients[-1].wait_for_server()

    def __make_state_subs(self):
        for cf_name in self.cf_names:
            tmp_sub = rospy.Subscriber('/' + cf_name + '/' + DEFAULT_CF_STATE_TOPIC, CrazyflieState, self.__state_cb)
            self.state_subs.append(tmp_sub)
            self.states.append(CrazyflieState())

    # ==================================================================================================================
    #
    #                                     C A L L B A C K  M E T H O D S  (T O P I C S)
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                  __P A C E _ 1 0 0 H Z _ S U B _ C A L L B A C K
    #
    # This callback is called whenever an Empty message is published by pace_100Hz_node; it is used to develop the main
    # routine.
    # ------------------------------------------------------------------------------------------------------------------
    def __pace_100Hz_sub_callback(self, msg):
        # Initializing states message:
        states = SwarmStates()
        states.states = self.states
        self.states_pub.publish(states)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                                       __S T A T E _ C B
    #
    # This callback is called whenever a CrazyflieState message is published by one crazyflie; this state is put in
    # correct position within states list.
    # ------------------------------------------------------------------------------------------------------------------
    def __state_cb(self, msg):
        # Getting id:
        ID = extractCfNumber(msg.name)

        # Update state vector:
        self.states[ID - 1] = msg

    # ==================================================================================================================
    #
    #                                 C A L L B A C K  M E T H O D S  (A C T I O N S)
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                             __S W A R M _ T A K E O F F _ A C T _ C A L L B A C K
    #
    # This method is used as the swarm takeoff action.
    # ------------------------------------------------------------------------------------------------------------------
    def __swarm_takeoff_act_callback(self, goal):
        # Output:
        feedback = TakeoffFeedback()
        result = TakeoffResult()
        result.result = True

        # Setting up takeoff request:
        _goal = TakeoffGoal()
        _goal.takeoff_height = goal.takeoff_height

        for takeoff_actn in self.takeoff_act_clients:
            takeoff_actn.send_goal(_goal)
            # print('\n\nTIPO:' + str(type(takeoff_actn)))
        # Vertex drone takeoff:
        takeoff_height = goal.takeoff_height + self.levels * self.vertical_offset
        _custom_goal = TakeoffGoal()
        _custom_goal.takeoff_height = takeoff_height
        self.takeoff_act_clients[0].send_goal(_custom_goal)
        prev_level_pos = 0
        for ii in range(1, self.levels + 1):
            amount_of_cfs_per_level = 2 * (ii + 1) + 2 * (ii - 1)
            for jj in range(prev_level_pos + 1, prev_level_pos + amount_of_cfs_per_level + 1):
                level_height = takeoff_height - ii * self.vertical_offset
                _custom_goal.takeoff_height = level_height
                self.takeoff_act_clients[jj].send_goal(_custom_goal)
            prev_level_pos = prev_level_pos + amount_of_cfs_per_level

        self.takeoff_act_clients[0].wait_for_result()
        self.__swarm_takeoff_act.set_succeeded(result)


    def __swarm_flocking_act_callback(self, goal):
        # Defining the goal:
        flock_goal = EmptyGoal()

        name = []
        # Sending the goal to all followers clients:
        for ii in range(0, len(self.flocking_act_clients)):
            self.flocking_act_clients[ii].send_goal(flock_goal)

        # Sending result:
        result = EmptyResult()
        result.executed = True

        self.__swarm_flocking_act.set_succeeded(result)

    def  __swarm_stop_act_callback(self, goal):
        result = EmptyResult()
        for stop_action in self.stop_clients:
            stop_action.send_goal(goal)
        result.executed = True
        self.__swarm_stop_act.set_succeeded(result)

    def __swarm_rel_vel_act_callback(self, goal):
        '''
            This action is used to perform a relative motion of the pyramid, like it was a rigid body. It is possible
            to perform two motions:
                1) Rigid translation (velocity components relative to pyramid body reference frame).
                2) Rotation around an axix.
            INPUT:
                - Type of axis:
                    Position.destination_info.desired_yaw = 0 => pyramid axis
                    Position.destination_info.desired_yaw = 1 => inertial axis, coordinates expressed in nest point

                - Rotation axis // to z inertial axis, placed in x_pos_axis, y_pos_axis (inertial frame):
                    x_pos_axis = Position.destination_info.desired_position.x
                    y_pos_axis = Position. destination_info.desired_position.y

                - Angular speed:
                    pyramid_angular_speed = Position.destination_info.desired_yaw_rate

                - Translational speed of pyramid's center of mass (related to local pyramid reference frame);
                    pyramid_vx_body = Position.destination_info.desired_velocity.x
                    pyramid_vy_body = Position.destination_info.desired_velocity.y
        '''
        # Result and check variables:
        result = Destination3DResult()
        result.result = True
        feasible_motion = True

        # Extracting data:
        desired_position = goal.destination_info
        duration = goal.time_duration

        desired_yaw_rate = desired_position.desired_yaw_rate

        # CASE 1: no rotational speed => ONLY TRANSLATION
        if desired_yaw_rate == 0:
            # Simply a rigid body translation => directly send the goal to each drone:
            for rel_vel_action in self.rel_vel_clients:
                rel_vel_action.send_goal(goal)
            self.rel_vel_clients[-1].wait_for_result()

        elif desired_yaw_rate != 0 and (desired_position.desired_velocity.x == 0 and desired_position.desired_velocity.y == 0 and desired_position.desired_velocity.z == 0):
            # CASE 2: Rotation around axis:

            # Saving actual states:
            actual_states = self.states

            # Determining position of rotational axis:
            if goal.destination_info.desired_yaw == 0:
                axis_position = Vector3(actual_states[0].position.x, actual_states[0].position.y, 0)
            else:
                axis_position = Vector3(goal.destination_info.desired_position.x,
                                        goal.destination_info.desired_position.y, 0)
            # List containing all goals:
            destination_goals = []

            # Determining the goal to be sent for each drone:
            for ii in range(0, len(actual_states)):
                # Message formulation for time duration and desired rotating speed:
                tmp_destination_goal = Destination3DGoal()
                tmp_destination_goal.time_duration = duration
                tmp_destination_goal.destination_info.desired_yaw_rate = desired_yaw_rate

                # Getting distance from axis:
                axis_distance = math.sqrt((actual_states[ii].position.x - axis_position.x) ** 2 + (
                            actual_states[ii].position.y - axis_position.y) ** 2)

                # Initial angle between axis distance vector and inertial x axis:
                psi_0 = atan2(actual_states[ii].position.y - axis_position.y,
                              actual_states[ii].position.x - axis_position.x)

                if goal.destination_info.desired_yaw == 0:
                    # Computing velocity components in body reference frame:
                    vx_rel = - deg2rad(desired_yaw_rate) * axis_distance * math.sin(psi_0)
                    vy_rel = deg2rad(desired_yaw_rate) * axis_distance * math.cos(psi_0)

                else:
                    # Computing velocity components in body reference frame:
                    vx_rel = deg2rad(desired_yaw_rate) * axis_distance * math.cos(psi_0)
                    vy_rel = deg2rad(desired_yaw_rate) * axis_distance * math.sin(psi_0)

                # Check if the required velocities respect the limits:
                if vx_rel > MAX_VELOCITY_X or vy_rel > MAX_VELOCITY_Y:
                    feasible_motion = False
                    break
                else:
                    tmp_destination_goal.destination_info.desired_velocity.x = vx_rel
                    tmp_destination_goal.destination_info.desired_velocity.y = vy_rel
                    destination_goals.append(tmp_destination_goal)

            # If the motion respect the limits on the velocity perform it, otherwise show error message:
            if feasible_motion:
                cont = 0
                for rel_vel_client in self.rel_vel_clients:
                    rel_vel_client.send_goal(destination_goals[cont])
                    cont += 1
                self.rel_vel_clients[-1].wait_for_result()
            else:
                result.result = False
                rospy.logerr('REQUIRED MOTION NOT FEASIBLE: velocity limit exceeded.')
        else:
            # CASE 3: not implemented:
            result.result = False
            rospy.logerr('MOTION NOT IMPLEMENTED')

        self.__swarm_rel_vel_act.set_succeeded(result)
    # ==================================================================================================================
    #
    #                          F E E D B A C K  C A L L B A C K  M E T H O D S  (A C T I O N S)
    #
    # ==================================================================================================================
    def __cf_takeoff_feedback_cb(self, feedback):
        pass