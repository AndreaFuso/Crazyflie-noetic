# ROS MODULES
import rospy
import actionlib

import math

# Action
from crazyflie_messages.msg import TakeoffAction, TakeoffGoal, TakeoffResult, TakeoffFeedback
from crazyflie_messages.msg import Destination3DAction, Destination3DGoal, Destination3DResult, Destination3DFeedback
from crazyflie_messages.msg import VelocityTrajectoryAction, VelocityTrajectoryGoal, VelocityTrajectoryResult, VelocityTrajectoryFeedback

from std_msgs.msg import Empty
from crazyflie_messages.msg import Position

from crazy_common_py.common_functions import deg2rad
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

        # List of clients for relative destination action per each drone:
        self.relative_motion_act_clients = []
        self.__make_relative_motion_clients()

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       S U B S C R I B E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       P U B L I S H E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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
        self.__swarm_traslation_act = actionlib.SimpleActionServer('/pyramid_swarm/traslation_actn', Destination3DAction,
                                                                   self.__swarm_traslation_act_callback, False)
        self.__swarm_traslation_act.start()

        # Action o perform circular motion:
        self.__circular_motion_act = actionlib.SimpleActionServer('/pyramid_swarm/circular_motion', Destination3DAction,
                                                                  self.__circular_motion_act_callback, False)
        self.__circular_motion_act.start()
        self.__velocity_trajectory_act_client = actionlib.SimpleActionClient('/cf1' + '/velocity_trajectory',
                                                                            VelocityTrajectoryAction)

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

    def __make_relative_motion_clients(self):
        for cf_name in self.cf_names:
            tmp_action = actionlib.SimpleActionClient('/' + cf_name + '/relative_position_3D_motion', Destination3DAction)
            self.relative_motion_act_clients.append(tmp_action)
            self.relative_motion_act_clients[-1].wait_for_server()

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

        # Setting up takeoff request:
        _goal = TakeoffGoal()
        _goal.takeoff_height = goal.takeoff_height

        for takeoff_actn in self.takeoff_act_clients:
            takeoff_actn.send_goal(_goal, feedback_cb=self.__cf_takeoff_feedback_cb)
            # print('\n\nTIPO:' + str(type(takeoff_actn)))
        # Vertex drone takeoff:
        takeoff_height = goal.takeoff_height + self.levels * self.vertical_offset
        _custom_goal = TakeoffGoal()
        _custom_goal.takeoff_height = takeoff_height
        self.takeoff_act_clients[0].send_goal(_custom_goal, feedback_cb=self.__cf_takeoff_feedback_cb)
        prev_level_pos = 0
        for ii in range(1, self.levels + 1):
            amount_of_cfs_per_level = 2 * (ii + 1) + 2 * (ii - 1)
            for jj in range(prev_level_pos + 1, prev_level_pos + amount_of_cfs_per_level + 1):
                level_height = takeoff_height - ii * self.vertical_offset
                _custom_goal.takeoff_height = level_height
                self.takeoff_act_clients[jj].send_goal(_custom_goal, feedback_cb=self.__cf_takeoff_feedback_cb)
            prev_level_pos = prev_level_pos + amount_of_cfs_per_level


        self.__swarm_takeoff_act.set_succeeded(result)

    def __swarm_traslation_act_callback(self, goal):
        # Output:
        feedback = Destination3DFeedback()
        result = Destination3DResult()

        for action in self.relative_motion_act_clients:
            action.send_goal(goal, feedback_cb=self.__cf_takeoff_feedback_cb)
        self.__swarm_traslation_act.set_succeeded(result)

    def __circular_motion_act_callback(self, goal):
        omega = goal.destination_info.desired_yaw
        r = 2.0
        dt = 1 / 100
        durantion = 2 * math.pi / omega
        time_istants = int(durantion/dt)

        positions = []
        t = 0.0

        for ii in range(0, time_istants):
            tmp_pos = Position()
            if omega * t >= 360:
                t = 0.0
            tmp_pos.desired_position.x = r * math.cos(omega * t)
            tmp_pos.desired_position.y = r * math.sin(omega * t)
            tmp_pos.desired_position.z = 0.5
            tmp_pos.desired_velocity.x = - r * omega * math.sin(omega * t)
            tmp_pos.desired_velocity.y = r * omega * math.cos(omega * t)
            tmp_pos.desired_velocity.z = 0.0
            tmp_pos.desired_yaw = omega * t + deg2rad(90)
            positions.append(tmp_pos)
            t += dt

        trajectory = VelocityTrajectoryGoal()
        trajectory.states_vector = positions
        trajectory.dt = dt

        self.__velocity_trajectory_act_client.send_goal(trajectory)




    # ==================================================================================================================
    #
    #                          F E E D B A C K  C A L L B A C K  M E T H O D S  (A C T I O N S)
    #
    # ==================================================================================================================
    def __cf_takeoff_feedback_cb(self, feedback):
        pass