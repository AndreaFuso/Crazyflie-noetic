# ROS MODULES
import rospy
import actionlib
# CUSTOM MODULES

from crazy_common_py.dataTypes import Vector3, CfStatus, MovementMode
from crazyflie_simulator.MotorControllerSim import MotorControllerSim
from crazyflie_simulator.FlightControllerSimCustom import FlightControllerCustom
from crazy_common_py.common_functions import rad2deg, deg2rad
from crazy_common_py.constants import *
from crazy_common_py.default_topics import DEFAULT_CF_STATE_TOPIC, DEFAULT_100Hz_PACE_TOPIC, DEFAULT_500Hz_PACE_TOPIC, \
    DEFAULT_MOTOR_CMD_TOPIC, DEFAULT_DESIRED_MOTOR_CMD_TOPIC, DEFAULT_ACTUAL_DESTINATION_TOPIC
from crazy_common_py.default_topics import DEFAULT_TAKEOFF_ACT_TOPIC, DEFAULT_LAND_ACT_TOPIC, DEFAULT_ABS_POS_TOPIC, \
    DEFAULT_REL_POS_TOPIC, DEFAULT_ABS_VEL_TOPIC, DEFAULT_REL_VEL_TOPIC, DEFAULT_STOP_TOPIC, DEFAULT_FLOCK_TOPIC#, DEFAULT_ACTUAL_MPC_TOPIC

from crazy_common_py.default_topics import DEFAULT_TAKEOFF_SRV_TOPIC, DEFAULT_LAND_SRV_TOPIC

from crazyflie_manager.NeighborSpotter import NeighborSpotter
from crazy_common_py.dataTypes import SphericalSpotter, Role

# OTHER MODULES
import time
import math
import numpy as np

# MESSAGES
# Topic
from crazyflie_messages.msg import Position, CrazyflieState, Attitude
from std_msgs.msg import Empty

# Service
from crazyflie_messages.srv import Takeoff_srv, Takeoff_srvResponse, Takeoff_srvRequest

# Action
from crazyflie_messages.msg import TakeoffAction, TakeoffGoal, TakeoffResult, TakeoffFeedback
from crazyflie_messages.msg import Destination3DAction, Destination3DGoal, Destination3DResult, Destination3DFeedback
from crazyflie_messages.msg import VelocityTrajectoryAction, VelocityTrajectoryGoal, VelocityTrajectoryResult, \
    VelocityTrajectoryFeedback
from crazyflie_messages.msg import EmptyAction, EmptyGoal, EmptyResult, EmptyFeedback
from crazyflie_messages.msg import GetStateAction, GetStateGoal, GetStateFeedback, GetStateResult

class MotionCommanderSim:
    # ==================================================================================================================
    #
    #                                               C O N S T R U C T O R
    #
    # This class is similar to the MotionCommander of official Bitcraze Python library: it collects high level commands.
    # INPUTS:
    #   1) cfName -> name of the crazyflie in the simulation;
    # ==================================================================================================================
    def __init__(self, cfName):
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                           P R O P E R T I E S  I N I T I A L I Z A T I O N
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Name of the virtual Crazyflie:
        self.name = cfName

        # Property to understand the actual status of the Crazyflie (default LANDED):
        self.status = CfStatus.LANDED

        # Instance of motor controller:
        self.motors_controller = MotorControllerSim(cfName)

        # Instance of flight controller:
        self.flight_controller = FlightControllerCustom(cfName)

        # Property to understand if movement actions have to be interrupted:
        self.stopActions = False

        # Spotter:
        if cfName != DEFAULT_LEADER:
            self.role = Role.FOLLOWER
            spotting_radius = DEFAULT_RADIUS_HORIZON
            spherical_spotter = SphericalSpotter(spotting_radius)
            self.spotter = NeighborSpotter(cfName, spherical_spotter)
        else:
            self.role = Role.LEADER

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                    S U B S C R I B E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Subscribers to perform operations at a certain frequency:
        self.pace_100Hz_sub = rospy.Subscriber('/' + DEFAULT_100Hz_PACE_TOPIC, Empty, self.__pace_100Hz_callback)
        self.pace_500Hz_sub = rospy.Subscriber('/' + DEFAULT_500Hz_PACE_TOPIC, Empty, self.__pace_500Hz_callback)

        # Subscriber to obtain the actual state of the virtual Crazyflie:
        self.actual_state_sub = rospy.Subscriber('/' + cfName + '/' + DEFAULT_CF_STATE_TOPIC, CrazyflieState,
                                                 self.__actual_state_sub_callback)
        self.actual_state = CrazyflieState()

        # Subscriber to obtain the desired motor command (final output of all the pids within FlightControllerSim):
        self.desired_motor_command_sub = rospy.Subscriber('/' + cfName + '/' + DEFAULT_DESIRED_MOTOR_CMD_TOPIC,
                                                          Attitude, self.__desired_motor_command_callback)
        self.desired_motor_command = Attitude()


        # Subscriber to read the desired velocity computed by the MPC controller
        self.mpc_velocity_sub = rospy.Subscriber('/' + cfName + '/mpc_velocity',
                                                          Position, self.__mpc_velocity_callback)
        self.mpc_velocity = Position()



        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       P U B L I S H E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Publisher to publish the desired destination (to be sent to FlightControllerSim):
        self.trajectory_pub = rospy.Publisher('/' + cfName + '/' + DEFAULT_ACTUAL_DESTINATION_TOPIC,
                                              Position, queue_size=1)
        self.position_target = Position()

        # # Publisher to publish the desired destination using MPC (to be sent to FlightControllerSim):
        # self.mpc_target_pub = rospy.Publisher('/' + cfName + '/mpc_target', Position, queue_size=1)
        # self.mpc_target = Position()


        # # Publisher to publish the desired destination using MPC (single integrator) (to be sent to FlightControllerSim):
        # self.mpc_single_pub = rospy.Publisher('/' + cfName + '/mpc_single', Position, queue_size=1)
        # self.mpc_single = Position()


        #TODO: aggiungere subscriber per traiettoria pubblicata dall'esterno, un'azione blocca la pubblicazione di
        # position_target e la sostituisce con quella letta dallo subscriber: questo funzionamento rimane in piedi
        # finche' l'azione di traiettoria non termina per tempo o per stop action; a quel punto si ritorna a
        # pubblicare position_target

        # Publisher to publish motor commands directly to the motors:
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # NOTE: "/set_destination_position" keeps publishing the desired position, and FlightControllerSim keeps
        # calculating the motor commands publishing them on "/set_desired_motion_command"; anyway it's the
        # MotionCommander that decides when to actually send commands to the motors.
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        self.motor_command_pub = rospy.Publisher('/' + cfName + '/' + DEFAULT_MOTOR_CMD_TOPIC, Attitude, queue_size=1)
        self.motor_command = Attitude()

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                           S E R V I C E S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Takeoff service (server + client):
        self.__takeoff_srv = rospy.Service('/' + cfName + '/' + DEFAULT_TAKEOFF_SRV_TOPIC,
                                           Takeoff_srv, self.__takeoff_srv_callback)
        self.__takeoff_srv_client = rospy.ServiceProxy('/' + cfName + '/' + DEFAULT_TAKEOFF_SRV_TOPIC, Takeoff_srv)

        # Land service:
        self.__land_srv = rospy.Service('/' + cfName + '/' + DEFAULT_LAND_SRV_TOPIC,
                                        Takeoff_srv, self.__land_srv_callback)
        self.__land_srv_client = rospy.ServiceProxy('/' + cfName + '/' + DEFAULT_LAND_SRV_TOPIC, Takeoff_srv)

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                           A C T I O N S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Takeoff action (server + client):
        self.__takeoff_act = actionlib.SimpleActionServer('/' + cfName + '/' + DEFAULT_TAKEOFF_ACT_TOPIC,
                                                          TakeoffAction, self.__takeoff_act_callback, False)
        self.__takeoff_act.start()
        # self.__takeoff_act_client = actionlib.SimpleActionClient('/' + cfName + '/' + DEFAULT_TAKEOFF_ACT_TOPIC,
        #                                                          TakeoffAction)

        # Landing action (server + client):
        self.__land_act = actionlib.SimpleActionServer('/' + cfName + '/' + DEFAULT_LAND_ACT_TOPIC,
                                                       TakeoffAction, self.__land_act_callback, False)
        self.__land_act.start()
        self.__land_act_client = actionlib.SimpleActionClient('/' + cfName + '/' + DEFAULT_LAND_ACT_TOPIC,
                                                              TakeoffAction)

        # Relative 3D displacement movement:
        self.__relative_3D_displacement_act = actionlib.SimpleActionServer('/' + cfName + '/' + DEFAULT_REL_POS_TOPIC,
                                                                           Destination3DAction,
                                                                           self.__relative_3D_displacement_act_callback,
                                                                           False)
        self.__relative_3D_displacement_act.start()
        self.__relative_3D_displacement_act_client = actionlib.SimpleActionClient('/' + cfName + '/' + DEFAULT_REL_POS_TOPIC, Destination3DAction)

        # Absolute 3D positioning movement:
        self.__absolute_position_3D_motion_act = actionlib.SimpleActionServer('/' + cfName + '/' + DEFAULT_ABS_POS_TOPIC,
                                                                              Destination3DAction,
                                                                              self.__absolute_position_3D_motion_act_callback,
                                                                              False)
        self.__absolute_position_3D_motion_act.start()

        # Relative 3D velocity motion:
        self.__relative_3D_velocity_motion_act = actionlib.SimpleActionServer('/' + cfName + '/' + DEFAULT_REL_VEL_TOPIC,
                                                                              Destination3DAction,
                                                                              self.__relative_3D_velocity_motion_act_callback,
                                                                              False)
        self.__relative_3D_velocity_motion_act.start()
        self.__relative_3D_velocity_motion_act_client = actionlib.SimpleActionClient('/' + cfName + '/' + DEFAULT_REL_VEL_TOPIC, Destination3DAction)

        # Absolute 3D velocity motion:
        self.__absolute_3D_velocity_motion_act = actionlib.SimpleActionServer('/' + cfName + '/' + DEFAULT_ABS_VEL_TOPIC,
                                                                              Destination3DAction,
                                                                              self.__absolute_3D_velocity_motion_act_callback,
                                                                              False)
        self.__absolute_3D_velocity_motion_act.start()

        # Stop action (with related client):
        self.__stop_act = actionlib.SimpleActionServer('/' + cfName + '/' + DEFAULT_STOP_TOPIC, EmptyAction,
                                                       self.__stop_act_callback, False)
        self.__stop_act.start()
        self.__stop_act_client = actionlib.SimpleActionClient('/' + cfName + '/' + DEFAULT_STOP_TOPIC, EmptyAction)

        # Leader - follower action:
        self.__flock_act = actionlib.SimpleActionServer('/' + cfName + '/' + DEFAULT_FLOCK_TOPIC, EmptyAction,
                                                        self.__flock_act_callback, False)
        self.__flock_act.start()


        # Make square position action:
        self.__square_position_act = actionlib.SimpleActionServer('/' + cfName + '/square_position_act', EmptyAction, 
                                                         self.__square_position_act_callback, False)
        self.__square_position_act.start()

        # Make square action:
        self.__square_velocity_act = actionlib.SimpleActionServer('/' + cfName + '/square_velocity_act', EmptyAction, 
                                                         self.__square_velocity_act_callback, False)
        self.__square_velocity_act.start()


        # # Use MPC controller to get to a desired state:
        # self.__mpc_target_act = actionlib.SimpleActionServer('/' + cfName + '/mpc_target_act', Destination3DAction, 
        #                                                  self.__mpc_target_act_callback, False)
        # self.__mpc_target_act.start()


        # # Use MPC controller to get to a desired state (single integrator):
        # self.__mpc_single_act = actionlib.SimpleActionServer('/' + cfName + '/mpc_single_act', Destination3DAction, 
        #                                                  self.__mpc_single_act_callback, False)
        # self.__mpc_single_act.start()



    # ==================================================================================================================
    #
    #                                     C A L L B A C K  M E T H O D S  (T O P I C S)
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                  __P A C E _ 1 0 0 H Z _ C A L L B A C K
    #
    # This callback is called every 1 / 100 seconds, is used to publish the desired position to the FlightController at
    # 100 Hz.
    # ------------------------------------------------------------------------------------------------------------------
    def __pace_100Hz_callback(self, msg):
        if self.status != CfStatus.LANDED:
            self.trajectory_pub.publish(self.position_target)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                  __P A C E _ 5 0 0 H Z _ C A L L B A C K
    #
    # This callback is called every 1 / 500 seconds, is used to publish the desired motor command input to the motors
    # at 500 Hz.
    # ------------------------------------------------------------------------------------------------------------------
    def __pace_500Hz_callback(self, msg):
        self.motor_command_pub.publish(self.desired_motor_command)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                       __D E S I R E D _ M O T O R _ C O M M A N D _ C A L L B A C K
    #
    # This callback saves the motor commands coming from the FLightControllerSim in a class' variable.
    # ------------------------------------------------------------------------------------------------------------------
    def __desired_motor_command_callback(self, msg):
        if self.status != CfStatus.LANDED:
            self.desired_motor_command.desired_attitude.roll = msg.desired_attitude.roll
            self.desired_motor_command.desired_attitude.pitch = msg.desired_attitude.pitch
            self.desired_motor_command.desired_attitude.yaw = msg.desired_attitude.yaw
            self.desired_motor_command.desired_thrust = msg.desired_thrust

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                               __M O T O R _ C O M M A N D _ C A L L B A C K
    #
    # ------------------------------------------------------------------------------------------------------------------
    def __motor_command_sub_callback(self, msg):
        pass

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                               __A C T U A L _ S T A T E _ S U B _ C A L L B A C K
    #
    # This callback updates the internal variable reffering to actual state of the Crazyflie.
    # ------------------------------------------------------------------------------------------------------------------
    def __actual_state_sub_callback(self, msg):
        self.actual_state.name = msg.name

        self.actual_state.position.x = msg.position.x
        self.actual_state.position.y = msg.position.y
        self.actual_state.position.z = msg.position.z

        self.actual_state.orientation.roll = msg.orientation.roll
        self.actual_state.orientation.pitch = msg.orientation.pitch
        self.actual_state.orientation.yaw = msg.orientation.yaw

        self.actual_state.velocity.x = msg.velocity.x
        self.actual_state.velocity.y = msg.velocity.y
        self.actual_state.velocity.z = msg.velocity.z

        self.actual_state.rotating_speed.x = msg.rotating_speed.x
        self.actual_state.rotating_speed.y = msg.rotating_speed.y
        self.actual_state.rotating_speed.z = msg.rotating_speed.z




   # ------------------------------------------------------------------------------------------------------------------
    #
    #                               __M P C _ V E L O C I T Y _ C A L L B A C K
    #
    # This callback gets the desired velocity computed by the mpc controller and sets the velocity target so that
    # the velocity command is provided to the velocity controller
    # ------------------------------------------------------------------------------------------------------------------
    def __mpc_velocity_callback(self, msg):
                
        # Setting velocity mode:
        self.flight_controller.mode = MovementMode.VELOCITY
        
        self.position_target.desired_velocity.x = msg.desired_velocity.x
        self.position_target.desired_velocity.y = msg.desired_velocity.y
        self.position_target.desired_velocity.z = msg.desired_velocity.z




    # ==================================================================================================================
    #
    #                                 C A L L B A C K  M E T H O D S  (S E R V I C E S)
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                    __T A K E O F F _ S R V _ C A L L B A C K
    #
    # This service is used to perform a vertical takeoff, its goal has one parameter:
    #   * takeoff_height -> target takeoff height [m] to be reached to switch crazyflie state from LAND to FLYING;
    # ------------------------------------------------------------------------------------------------------------------
    def __takeoff_srv_callback(self, request):
        # Setting up new status:
        self.status = CfStatus.TAKING_OFF

        # Getting takeoff height:
        takeoff_height = request.takeoff_height

        # Getting initial position:
        initial_position = Vector3(self.actual_state.position.x, self.actual_state.position.y, self.actual_state.position.z)
        initial_attitude = rad2deg(self.actual_state.orientation.yaw)

        # Setting up output:
        response = Takeoff_srvResponse()
        response.result = False

        # Returning true when the Crazyflie has reached the desired height:
        rate = rospy.Rate(100)
        while True:
            # Getting current state:
            actual_state = self.actual_state

            # Verify if the Crazyflie has reached the takeoff height:
            if math.fabs(takeoff_height - actual_state.position.z) <= 0.005:
                response.result = True
                break
            # Sending commands to reach the takeoff height:
            self.position_target.desired_position.x = initial_position.x
            self.position_target.desired_position.y = initial_position.y
            self.position_target.desired_position.z = takeoff_height
            self.position_target.desired_yaw = initial_attitude

            # Telling MotorControllerSim that can send motor commands:
            self.motors_controller.canSend = True

            rate.sleep()

        # Updating status:
        self.status = CfStatus.FLYING

        return response

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                    __L A N D _ S R V _ C A L L B A C K
    #
    # This service is used to perform a vertical landing, its goal has one parameter:
    #   * takeoff_height -> height [m] at which the crazyflie has to move before turning off motors;
    # ------------------------------------------------------------------------------------------------------------------
    def __land_srv_callback(self, request):
        # Getting takeoff height:
        landing_height = request.takeoff_height

        # Getting initial position:
        initial_position = Vector3(self.actual_state.position.x, self.actual_state.position.y,
                                   self.actual_state.position.z)
        initial_attitude = rad2deg(self.actual_state.orientation.yaw)

        # Setting up output:
        response = Takeoff_srvResponse()
        response.result = False

        # Returning true when the Crazyflie has reached the desired height:
        rate = rospy.Rate(100)
        while True:
            # Getting current state:
            actual_state = self.actual_state

            # Verify if the Crazyflie has reached the takeoff height:
            if math.fabs(landing_height - actual_state.position.z) <= 0.005:
                response.result = True
                break
            # Sending commands to reach the takeoff height:
            self.position_target.desired_position.x = initial_position.x
            self.position_target.desired_position.y = initial_position.y
            self.position_target.desired_position.z = landing_height
            self.position_target.desired_yaw = initial_attitude

            # Telling MotorControllerSim that can send motor commands:
            self.motors_controller.canSend = True

            rate.sleep()

        # Updating status:
        self.status = CfStatus.LANDED

        # Telling MotorControllerSim that can send motor commands:
        self.motors_controller.canSend = False

        return response


    # ==================================================================================================================
    #
    #                                 C A L L B A C K  M E T H O D S  (A C T I O N S)
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                    __T A K E O F F _ A C T _ C A L L B A C K
    #
    # This action is used to perform a vertical takeoff, its goal has one parameter:
    #   * takeoff_height -> target takeoff height [m] to be reached to switch crazyflie state from LAND to FLYING;
    # ------------------------------------------------------------------------------------------------------------------
    def __takeoff_act_callback(self, goal):
        # Setting up new status:
        self.status = CfStatus.TAKING_OFF

        # Rate definition:
        rate = rospy.Rate(100)
        success = True

        # Output:
        feedback = TakeoffFeedback()
        result = TakeoffResult()

        # Getting initial position:
        initial_position = Vector3(self.actual_state.position.x, self.actual_state.position.y,
                                   self.actual_state.position.z)
        initial_attitude = rad2deg(self.actual_state.orientation.yaw)

        # Getting target takeoff height:
        takeoff_height = goal.takeoff_height

        while True:
            # Getting current state:
            actual_state = self.actual_state
            
            absolute_distance = math.fabs(takeoff_height - actual_state.position.z)

            # Publishing absolute distance as feedback:
            feedback.absolute_distance = absolute_distance
            self.__takeoff_act.publish_feedback(feedback)

            # Verify if the Crazyflie has reached the takeoff height:
            if absolute_distance <= 0.005:
                success = True
                break

            # Check preemption:
            if self.__takeoff_act.is_preempt_requested():
                success = False
                info_msg = 'Takeoff action canceled for ' + self.name
                rospy.loginfo(info_msg)
                result.result = False
                self.__takeoff_act.set_preempted()
                break

            # Sending commands to reach the takeoff height:
            self.position_target.desired_position.x = initial_position.x
            self.position_target.desired_position.y = initial_position.y
            self.position_target.desired_position.z = takeoff_height
            self.position_target.desired_yaw = initial_attitude

            # Telling MotorControllerSim that can send motor commands:
            self.motors_controller.canSend = True

            rate.sleep()

        if success:
            self.status = CfStatus.FLYING
            result.result = True
            self.__takeoff_act.set_succeeded(result)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                    __L A N D _ A C T _ C A L L B A C K
    #
    # This action is used to perform a vertical landing, its goal has one parameter:
    #   * takeoff_height -> height [m] at which the crazyflie has to move before turning off motors;
    # ------------------------------------------------------------------------------------------------------------------
    def __land_act_callback(self, goal):
        # Rate definition:
        rate = rospy.Rate(100.0)
        success = True

        # Output:
        feedback = TakeoffFeedback()
        result = TakeoffResult()

        # Getting initial position:
        initial_position = Vector3(self.actual_state.position.x, self.actual_state.position.y,
                                   self.actual_state.position.z)
        initial_attitude = rad2deg(self.actual_state.orientation.yaw)

        # Getting target takeoff height:
        landing_height = goal.takeoff_height

        while True:
            # Getting current state:
            actual_state = self.actual_state

            # Aboslute distance:
            absolute_distance = math.fabs(landing_height - actual_state.position.z)

            # Publishing absolute distance as feedback:
            feedback.absolute_distance = absolute_distance
            self.__land_act.publish_feedback(feedback)

            # Verify if the Crazyflie has reached the takeoff height:
            if absolute_distance <= 0.005:
                success = True
                break

            # Check preemption:
            if self.__land_act.is_preempt_requested():
                success = False
                info_msg = 'Landing action canceled for ' + self.name
                rospy.loginfo(info_msg)
                result.result = False
                self.__land_act.set_preempted()
                break

            # Sending commands to reach the takeoff height:
            self.position_target.desired_position.x = initial_position.x
            self.position_target.desired_position.y = initial_position.y
            self.position_target.desired_position.z = landing_height
            self.position_target.desired_yaw = initial_attitude

            rate.sleep()

        if success:
            result.result = True
            self.status = CfStatus.LANDED
            self.motors_controller.canSend = False
            self.__land_act.set_succeeded(result)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                             __R E L A T I V E _ 3 D _ D I S P L A C E M E N T _ A C T _ C A L L B A C K
    #
    # This action perform a 3D motion relative to local crazyflie's reference system:
    #   * +x -> forward;
    #   * +y -> leftward;
    # with this action there's no control with respect velocity (computed by FlightControllerSim).
    # ------------------------------------------------------------------------------------------------------------------
    def __relative_3D_displacement_act_callback(self, goal):
        success = True
        # Setting up position mode:
        self.flight_controller.mode = MovementMode.POSITION

        # Getting actual state:
        initial_state = self.actual_state
        initial_yaw = initial_state.orientation.yaw   # [rad]

        # Computing final destination:
        delta_x_rel = goal.destination_info.desired_position.x
        delta_y_rel = goal.destination_info.desired_position.y
        delta_z_rel = goal.destination_info.desired_position.z
        delta_yaw_rel = goal.destination_info.desired_yaw   # [deg]

        destination_x = initial_state.position.x + delta_x_rel * math.cos(initial_yaw) - delta_y_rel * math.sin(initial_yaw)
        destination_y = initial_state.position.y + delta_x_rel * math.sin(initial_yaw) + delta_y_rel * math.cos(initial_yaw)
        destination_z = initial_state.position.z + delta_z_rel

        # Output:
        feedback = Destination3DFeedback()
        result = Destination3DResult()

        # Check preemption:
        if self.__relative_3D_displacement_act.is_preempt_requested():
            success = False
            info_msg = 'Destination canceled for ' + self.name
            rospy.loginfo(info_msg)
            result.result = False
            self.__relative_3D_displacement_act.set_preempted()
            return

        # Sending commands to reach the desired point:
        self.position_target.desired_position.x = destination_x
        self.position_target.desired_position.y = destination_y
        self.position_target.desired_position.z = destination_z

        self.position_target.desired_yaw = rad2deg(initial_yaw) + delta_yaw_rel

        # self.position_target.desired_velocity.x = goal.destination_info.desired_velocity.x
        # self.position_target.desired_velocity.y = goal.destination_info.desired_velocity.y
        # self.position_target.desired_velocity.z = goal.destination_info.desired_velocity.z

        # while True:
        #     # Getting current state:
        #     self.diff_x = destination_x - self.actual_state.position.x
        #     self.diff_y = destination_y - self.actual_state.position.y
        #     self.diff_z = destination_z - self.actual_state.position.z

        #     if self.diff_x < 0.005 and self.diff_y < 0.005 and self.diff_z < 0.005:
        #         success = True
        #         break



        #     # Verify if the Crazyflie has reached the target:
        #     if absolute_distance <= 0.005:
        #         # success = True
        #         # break
        #         result.result = True
        #         self.__relative_3D_displacement_act.set_succeeded(result)

            
        if success:
            result.result = True
            self.__relative_3D_displacement_act.set_succeeded(result)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                  __A B S O L U T E _ P O S I T I O N _ 3 D _ M O T I O N _ A C T _ C A L L B A C K
    #
    # This action perform a 3D motion to move to the absolute 3D destination position; wit this action there's no
    # control with respect velocity (computed by FlightControllerSim).
    # ------------------------------------------------------------------------------------------------------------------
    def __absolute_position_3D_motion_act_callback(self, goal):
        success = True
        # Setting up position mode:
        self.flight_controller.mode = MovementMode.POSITION

        # Getting actual state:
        actual_state = self.actual_state

        # Computing final destination:
        destination_x = goal.destination_info.desired_position.x
        destination_y = goal.destination_info.desired_position.y
        destination_z = goal.destination_info.desired_position.z

        # Output:
        feedback = Destination3DFeedback()
        result = Destination3DResult()

        # Check preemption:
        if self.__absolute_position_3D_motion_act.is_preempt_requested():
            success = False
            info_msg = 'Destination canceled for ' + self.name
            rospy.loginfo(info_msg)
            result.result = False
            self.__absolute_position_3D_motion_act.set_preempted()
            return

        # Sending commands to reach the desired point:
        self.position_target.desired_position.x = destination_x
        self.position_target.desired_position.y = destination_y
        self.position_target.desired_position.z = destination_z

        self.position_target.desired_yaw = goal.destination_info.desired_yaw

        self.position_target.desired_velocity.x = goal.destination_info.desired_velocity.x
        self.position_target.desired_velocity.y = goal.destination_info.desired_velocity.y
        self.position_target.desired_velocity.z = goal.destination_info.desired_velocity.z

        if success:
            result.result = True
            self.__absolute_position_3D_motion_act.set_succeeded(result)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                 __A B S O L U T E _ 3 D _ V E L O C I T Y _ M O T I O N _  A C T _ C A L L B A C K
    #
    # This action makes crazyflie tracking an absolute velocity for a certain time duration [s]:
    #   * time_duration = 0  -> endless;
    #   * time_duration != 0 -> fixed duration;
    #   * desired_yaw -> desired yaw angular speed [deg/s];
    # ------------------------------------------------------------------------------------------------------------------
    def __absolute_3D_velocity_motion_act_callback(self, goal):
        success = True

        # Output & feedback:
        feedback = Destination3DFeedback()
        result = Destination3DResult()

        # Defining update rate in changing desired velocity:
        rate = rospy.Rate(100)

        # Understanding if it's endless action or fixed time duration:
        time_duration = goal.time_duration

        # Getting initial time and time duration (used for fixed time duration request):
        t0 = rospy.get_rostime()
        ros_time_duration = rospy.Duration.from_sec(goal.time_duration)
        final_rostime = t0 + ros_time_duration

        # Setting velocity mode:
        self.flight_controller.mode = MovementMode.VELOCITY

        while True:
            # In case of fixed time duration check if desired time has elapsed:
            if time_duration != 0:
                # Getting current time:
                actual_time = rospy.get_rostime()

                # Remaining time:
                remaining_time = (final_rostime - actual_time).to_sec()

                # Publishing absolute distance as feedback:
                feedback.remaining_time = remaining_time
                self.__absolute_3D_velocity_motion_act.publish_feedback(feedback)

                # If time duration has elapsed exit the cycle:
                if remaining_time <= 0:
                    success = True
                    break

            # Sending commands to follow the desired velocity and yaw attitude rate:
            self.position_target.desired_velocity.x = goal.destination_info.desired_velocity.x
            self.position_target.desired_velocity.y = goal.destination_info.desired_velocity.y
            self.position_target.desired_velocity.z = goal.destination_info.desired_velocity.z
            self.position_target.desired_yaw = goal.destination_info.desired_yaw

            # Check for preemption:
            if self.__absolute_3D_velocity_motion_act.is_preempt_requested() or self.stopActions:
                success = False
                info_msg = 'Absolute velocity motion canceled for ' + self.name
                rospy.loginfo(info_msg)
                result.result = False
                self.__absolute_3D_velocity_motion_act.set_preempted()
                break
            rate.sleep()

        # Calling stop action:
        stop_goal = EmptyGoal()
        self.__stop_act_client.send_goal(stop_goal, feedback_cb=self.__stop_act_client_feedback_cb)

        # Send the result:
        if success:
            result.result = True
        else:
            result.result = False

        self.__absolute_3D_velocity_motion_act.set_succeeded(result)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                 __R E L A T I V E _ 3 D _ V E L O C I T Y _ M O T I O N _  A C T _ C A L L B A C K
    #
    # This action makes crazyflie tracking a relative velocity (relative reference frame) for a certain time
    # duration [s]:
    #   * time_duration = 0  -> endless;
    #   * time_duration != 0 -> fixed duration;
    #   * desired_yaw -> desired yaw angular speed [deg/s];
    # ------------------------------------------------------------------------------------------------------------------
    def __relative_3D_velocity_motion_act_callback(self, goal):
        success = True

        # Output & feedback:
        feedback = Destination3DFeedback()
        result = Destination3DResult()

        # Defining update rate in changing desired velocity:
        rate = rospy.Rate(100)

        # Understanding if it's endless action or fixed time duration:
        time_duration = goal.time_duration

        # Getting initial time and time duration (used for fixed time duration request):
        t0 = rospy.get_rostime()
        ros_time_duration = rospy.Duration.from_sec(goal.time_duration)
        final_rostime = t0 + ros_time_duration

        # Setting velocity mode:
        self.flight_controller.mode = MovementMode.VELOCITY

        while True:
            # In case of fixed time duration check if desired time has elapsed:
            if time_duration != 0:
                # Getting current time:
                actual_time = rospy.get_rostime()

                # Remaining time:
                remaining_time = (final_rostime - actual_time).to_sec()

                # Publishing absolute distance as feedback:
                feedback.remaining_time = remaining_time
                self.__relative_3D_velocity_motion_act.publish_feedback(feedback)

                # If time duration has elapsed exit the cycle:
                if remaining_time <= 0:
                    success = True
                    break

            # Getting actual state:
            actual_state = self.actual_state

            # Getting relative velocity components:
            vx_rel = goal.destination_info.desired_velocity.x
            vy_rel = goal.destination_info.desired_velocity.y
            vz_rel = goal.destination_info.desired_velocity.z
            actual_yaw = actual_state.orientation.yaw

            # Compute absolute velocity components:
            vx_abs = vx_rel * math.cos(actual_yaw) - vy_rel * math.sin(actual_yaw)
            vy_abs = vx_rel * math.sin(actual_yaw) + vy_rel * math.cos(actual_yaw)
            vz_abs = vz_rel

            # Sending commands to follow the desired velocity and yaw attitude rate:
            self.position_target.desired_velocity.x = vx_abs
            self.position_target.desired_velocity.y = vy_abs
            self.position_target.desired_velocity.z = vz_abs
            self.position_target.desired_yaw_rate = goal.destination_info.desired_yaw_rate

            # Check for preemption:
            if self.__relative_3D_velocity_motion_act.is_preempt_requested() or self.stopActions:
                success = False
                info_msg = 'Relative velocity motion canceled for ' + self.name
                rospy.loginfo(info_msg)
                result.result = False
                self.__absolute_3D_velocity_motion_act.set_preempted()
                break
            rate.sleep()

        # Calling stop action:
        stop_goal = EmptyGoal()
        self.__stop_act_client.send_goal(stop_goal)
        self.__stop_act_client.wait_for_result()

        # Send the result:
        if success:
            result.result = True
        else:
            result.result = False
        #print('\n\nFINE MOVIMENTO\n\n')
        self.__relative_3D_velocity_motion_act.set_succeeded(result)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                          __S T O P _  A C T _ C A L L B A C K
    #
    # This action is used to stop the crazyflie:
    #   * if it's hovering in a certain point, nothing changes;
    #   * if it's moving, its speed reference is set to 0.0 in all directions; when the reference velocity is reached,
    #     its actual position is set as reference => switch to POSITION mode.
    # ------------------------------------------------------------------------------------------------------------------
    def __stop_act_callback(self, goal):
        deb_msg = 'Crazyflie ' + self.name + ' has received a STOP request!'
        rospy.logdebug(deb_msg)

        self.stopActions = True

        # Output:
        result = EmptyResult()
        feedback = EmptyFeedback()

        # Setting mode to VELOCITY:
        self.flight_controller.mode = MovementMode.VELOCITY

        # Getting actual velocity:
        actual_state = self.actual_state
        actual_vx = actual_state.velocity.x
        actual_vy = actual_state.velocity.y
        actual_vz = actual_state.velocity.z
        actual_yaw_rate = rad2deg(actual_state.rotating_speed.z)

        # Computing feedback value:
        feedback.feedback_value = math.sqrt(actual_vx ** 2 + actual_vy ** 2 + actual_vz ** 2 + actual_yaw_rate ** 2)
        self.__stop_act.publish_feedback(feedback)

        # Setting reference velocity to 0.0 in all directions:
        self.position_target.desired_velocity.x = 0.0
        self.position_target.desired_velocity.y = 0.0
        self.position_target.desired_velocity.z = 0.0
        self.position_target.desired_yaw_rate = 0.0

        rate = rospy.Rate(100)
        while feedback.feedback_value >= 0.05:
            # Updating actual velocity:
            actual_state = self.actual_state
            actual_vx = actual_state.velocity.x
            actual_vy = actual_state.velocity.y
            actual_vz = actual_state.velocity.z
            actual_yaw_rate = rad2deg(actual_state.rotating_speed.z)

            # Bigger threshold for yaw rate:
            if (math.sqrt(actual_vx ** 2 + actual_vy ** 2 + actual_vz ** 2) <= 0.05) and (actual_yaw_rate <= 0.05):
                feedback.feedback_value = 0.0
                self.__stop_act.publish_feedback(feedback)
            else:
                # Updating feedback value:
                feedback.feedback_value = math.sqrt(actual_vx ** 2 + actual_vy ** 2 + actual_vz ** 2 + actual_yaw_rate ** 2)
                self.__stop_act.publish_feedback(feedback)

            # Setting reference velocity to 0.0 in all directions:
            self.position_target.desired_velocity.x = 0.0
            self.position_target.desired_velocity.y = 0.0
            self.position_target.desired_velocity.z = 0.0
            self.position_target.desired_yaw_rate = 0.0

            rate.sleep()

        # Once the crazyflie is stopped let's set actual position as target position:
        actual_state = self.actual_state
        self.position_target.desired_position.x = actual_state.position.x
        self.position_target.desired_position.y = actual_state.position.y
        self.position_target.desired_position.z = actual_state.position.z
        self.position_target.desired_yaw = rad2deg(actual_state.orientation.yaw)

        # Setting back mode to POSITION:
        self.flight_controller.mode = MovementMode.POSITION

        # Set result:
        result.executed = True
        #print('\n\nFERMATO\n\n')
        self.__stop_act.set_succeeded(result)
        self.stopActions = False

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                        __F L O C K _ A C T _ C A L L B A C K
    #
    # This action perform a flocking action: if the considered crazyflie is a FOLLOWER it follows the rules defined
    # by the flock, while if it is a LEADER its target position is not changed. The Spotter calculate the desired
    # velocity and saves it in a public property, this action takes the desired velocity and sends it to the
    # FlightControllerSim (for further details see: crazyflie_manager/NeighborSpotter.py in crazy_common_py pkg).
    # ------------------------------------------------------------------------------------------------------------------
    def __flock_act_callback(self, msg):
        # Check crazyflie role:
        if self.role == Role.LEADER:
            warn_msg = self.name + ' has been requested to follow the flock, but it\'s a LEADER!'
            rospy.logwarn(warn_msg)
        # Activate the spotter;
        self.spotter.isActivated = True

        # Setting up the rate (100Hz):
        rate = rospy.Rate(100)

        # Setting mode to VELOCITY:
        self.flight_controller.mode = MovementMode.VELOCITY

        while True and not self.stopActions:
            # Setting the desired velocity equal to the one coming from the spotter:
            self.position_target.desired_velocity.x = self.spotter.desired_velocity.x
            self.position_target.desired_velocity.y = self.spotter.desired_velocity.y
            self.position_target.desired_velocity.z = self.spotter.desired_velocity.z

            # Check preemption:
            if self.__flock_act.is_preempt_requested() or self.stopActions:
                info_msg = self.name + ' flocking interrupted!'
                rospy.loginfo(info_msg)
                break

            rate.sleep()

        # Stopping spotter:
        self.spotter.isActivated = False

        # Calling stop action:
        stop_goal = EmptyGoal()
        self.__stop_act_client.send_goal(stop_goal, feedback_cb=self.__stop_act_client_feedback_cb)


        # Result:
        result = EmptyResult()
        result.executed = True

        self.__flock_act.set_succeeded(result)




    # ------------------------------------------------------------------------------------------------------------------
    #
    #                              __S Q U A R E _ P O S I T I O N _ A C T _ C A L L B A C K
    #
    # This action makes a crazyflie perform a square
    #
    # ------------------------------------------------------------------------------------------------------------------




    def __square_position_act_callback(self, goal):
        
        # Check if the crazyflie is flying
        if self.status == CfStatus.FLYING:
            i = 0

            success = True
            # Setting up position mode:
            self.flight_controller.mode = MovementMode.POSITION
            rate = rospy.Rate(100)

            while i < 4:
                # going forward (1 meter)
                self.forward_goal = Destination3DGoal()
                self.forward_goal.destination_info.desired_position.x = 1
                self.forward_goal.destination_info.desired_position.y = 0
                self.forward_goal.destination_info.desired_position.z = 0
                self.forward_goal.destination_info.desired_yaw = 0
                self.forward_goal.time_duration = 0.0
                
                self.__relative_3D_displacement_act_client.send_goal(self.forward_goal)
                self.__relative_3D_displacement_act_client.wait_for_result()

                while True:
                    # Getting current state:
                    self.diff_x = self.position_target.desired_position.x - self.actual_state.position.x
                    self.diff_y = self.position_target.desired_position.y - self.actual_state.position.y
                    self.diff_z = self.position_target.desired_position.z - self.actual_state.position.z
                    rate.sleep()
                    if abs(self.diff_x) < 0.005 and abs(self.diff_y) < 0.005 and abs(self.diff_z) < 0.005:
                        # success = True
                        break
                # rospy.loginfo('Finished the straight path number %f', i)
                print('Finished the straight path number ', i)
                print('The values of the errors are: ', self.diff_x, ', ', self.diff_y, ', ', self.diff_z)
                

                
                # rotate 90 degrees
                self.curve_goal = Destination3DGoal()
                self.curve_goal.destination_info.desired_position.x = 0
                self.curve_goal.destination_info.desired_position.y = 0
                self.curve_goal.destination_info.desired_position.z = 0
                self.curve_goal.destination_info.desired_yaw = 90
                self.curve_goal.time_duration = 0.0

                self.__relative_3D_displacement_act_client.send_goal(self.curve_goal)
                self.__relative_3D_displacement_act_client.wait_for_result()


                while True:
                    # Getting current state:
                    self.diff_yaw = self.position_target.desired_yaw - rad2deg(self.actual_state.orientation.yaw)
                    while abs(self.diff_yaw) > 360:
                        if self.diff_yaw > 360:
                            self.diff_yaw -= 360
                        else:
                            self.diff_yaw +=360
                    self.yawrate = self.actual_state.rotating_speed.z
                    # print(self.diff_yaw)
                    rate.sleep()
                    if abs(self.diff_yaw) < 0.5 and abs(self.yawrate) < 0.2:
                        # success = True
                        break
                # rospy.loginfo('Finished the rotation number %f', i)
                print('Finished the rotation number, ', i)
                print('The values of the errors are: ', self.diff_yaw, ', ', self.yawrate)
               
                i += 1

        else:
            pass


    # ------------------------------------------------------------------------------------------------------------------
    #
    #                             __S Q U A R E _ V E L O C I T Y _ A C T _ C A L L B A C K
    #
    # This action makes a crazyflie perform a square
    #
    # ------------------------------------------------------------------------------------------------------------------




    def __square_velocity_act_callback(self, goal):
        # Setting the goals
        rate = rospy.Rate(100)
        self.forward_goal = Destination3DGoal()
        self.forward_goal.destination_info.desired_velocity.x = 0.5
        self.forward_goal.destination_info.desired_velocity.y = 0
        self.forward_goal.destination_info.desired_velocity.z = 0
        self.forward_goal.destination_info.desired_yaw_rate = 0
        self.forward_goal.time_duration = 2.0

        self.curve_goal = Destination3DGoal()
        self.curve_goal.destination_info.desired_velocity.x = 0.2
        self.curve_goal.destination_info.desired_velocity.y = 0
        self.curve_goal.destination_info.desired_velocity.z = 0
        self.curve_goal.destination_info.desired_yaw_rate = 45
        self.curve_goal.time_duration = 2.0

        # Check if the crazyflie is flying
        if self.status == CfStatus.FLYING:
            i = 0
            while i < 4:
                # going forward
                self.__relative_3D_velocity_motion_act_client.send_goal(self.forward_goal)
                self.__relative_3D_velocity_motion_act_client.wait_for_result()
                rate.sleep()                
                # rotate 90 degrees
                self.__relative_3D_velocity_motion_act_client.send_goal(self.curve_goal)
                self.__relative_3D_velocity_motion_act_client.wait_for_result()
                rate.sleep()
                i += 1

        else:
            pass


    # # ------------------------------------------------------------------------------------------------------------------
    # #
    # #                             __M P C _ T A R G E T _ A C T _ C A L L B A C K
    # #
    # # This action sets the position target of the 3D MPC controller, including obstacle avoidance
    # #
    # # ------------------------------------------------------------------------------------------------------------------




    # def __mpc_target_act_callback(self, goal):

    #     success = True

    #     # Output & feedback:
    #     feedback = Destination3DFeedback()
    #     result = Destination3DResult()

    #     # Defining update rate in changing desired target:
    #     rate = rospy.Rate(100)

    #     # Setting up velocity mode (desired velocity is the output of the MPC controller):
    #     self.flight_controller.mode = MovementMode.VELOCITY
        
    #     # Setting the position target to be reached by the drone
    #     self.mpc_target.desired_position.x = goal.destination_info.desired_position.x
    #     self.mpc_target.desired_position.y = goal.destination_info.desired_position.y
    #     self.mpc_target.desired_position.z = goal.destination_info.desired_position.z


    #     # Here we start the loop that allows us to follow the desired velocity

    #     while(True):
    #         # Publishing the target on /cf1/mpc_target
    #         self.mpc_target_pub.publish(self.mpc_target)

    #         # Updating the position target with the one computed by the mpc controller
    #         self.position_target = self.flight_controller.position_target

    #         # Check for preemption:
    #         if self.__mpc_target_act.is_preempt_requested() or self.stopActions:
    #             success = False
    #             info_msg = 'MPC motion canceled for ' + self.name
    #             rospy.loginfo(info_msg)
    #             result.result = False
    #             self.__mpc_target_act.set_preempted()
    #             break

    #         elif (abs(self.actual_state.position.x - self.mpc_target.desired_position.x) < 0.01 and 
    #               abs(self.actual_state.position.y - self.mpc_target.desired_position.y) < 0.01 and 
    #               abs(self.actual_state.position.z - self.mpc_target.desired_position.z) < 0.01):
    #             info_msg = 'MPC motion successful for ' + self.name
    #             rospy.loginfo(info_msg)
    #             break

    #         else:
    #             pass

    #         rate.sleep()

        

    #     # Calling stop action:
    #     stop_goal = EmptyGoal()
    #     self.__stop_act_client.send_goal(stop_goal, feedback_cb=self.__stop_act_client_feedback_cb)

    #     # Send the result:
    #     if success:
    #         result.result = True
    #     else:
    #         result.result = False

    #     self.__mpc_target_act.set_succeeded(result)




    # # ------------------------------------------------------------------------------------------------------------------
    # #
    # #                             __M P C _ S I N G L E _ A C T _ C A L L B A C K
    # #
    # # This action sets the position target of the 3D MPC controller, including obstacle avoidance
    # #
    # # ------------------------------------------------------------------------------------------------------------------




    # def __mpc_single_act_callback(self, goal):

    #     success = True

    #     # Output & feedback:
    #     feedback = Destination3DFeedback()
    #     result = Destination3DResult()

    #     # Defining update rate in changing desired target:
    #     rate = rospy.Rate(100)

    #     # Setting up velocity mode (desired velocity is the output of the MPC controller):
    #     self.flight_controller.mode = MovementMode.VELOCITY
        
    #     # Setting the position target to be reached by the drone
    #     self.mpc_single.desired_position.x = goal.destination_info.desired_position.x
    #     self.mpc_single.desired_position.y = goal.destination_info.desired_position.y
    #     self.mpc_single.desired_position.z = goal.destination_info.desired_position.z


    #     # Here we start the loop that allows us to follow the desired velocity

    #     while(True):
    #         # Publishing the target on /cf1/mpc_single
    #         self.mpc_single_pub.publish(self.mpc_single)

    #         # Updating the position target with the one computed by the mpc controller
    #         self.position_target = self.flight_controller.position_target

    #         # Check for preemption:
    #         if self.__mpc_single_act.is_preempt_requested() or self.stopActions:
    #             success = False
    #             info_msg = 'MPC motion canceled for ' + self.name
    #             rospy.loginfo(info_msg)
    #             result.result = False
    #             self.__mpc_single_act.set_preempted()
    #             break

    #         elif (abs(self.actual_state.position.x - self.mpc_single.desired_position.x) < 0.01 and 
    #               abs(self.actual_state.position.y - self.mpc_single.desired_position.y) < 0.01 and 
    #               abs(self.actual_state.position.z - self.mpc_single.desired_position.z) < 0.01):
    #             info_msg = 'MPC motion successful for ' + self.name
    #             rospy.loginfo(info_msg)
    #             break

    #         else:
    #             pass

    #         rate.sleep()

        

    #     # Calling stop action:
    #     stop_goal = EmptyGoal()
    #     self.__stop_act_client.send_goal(stop_goal, feedback_cb=self.__stop_act_client_feedback_cb)

    #     # Send the result:
    #     if success:
    #         result.result = True
    #     else:
    #         result.result = False

    #     self.__mpc_target_act.set_succeeded(result)




    # ==================================================================================================================
    #
    #                            F E E D B A C K  C A L L B A C K  M E T H O D S  (A C T I O N S)
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                               __S T O P _ A C T _ C L I E N T _ F E E D B A C K _ C B
    #
    # Callback function for the feedback of stop action client.
    # ------------------------------------------------------------------------------------------------------------------

    def __stop_act_client_feedback_cb(self, feedback):
        message = 'Actual delta velocity for crazyflie ' + self.name + ' = ' + str(
            feedback.feedback_value) + ' [m/s]'
        rospy.logdebug(message)



    # ------------------------------------------------------------------------------------------------------------------
    #
    #                           __T A K E O F F _ A C T _ C L I E N T _ F E E D B A C K _ C B
    #
    # Callback function for the feedback of takeoff action client.
    # ------------------------------------------------------------------------------------------------------------------
    def __takeoff_act_client_feedback_cb(self, feedback):
        message = self.name + ' is taking off; actual absolute distance from target height: ' + str(
            feedback.absolute_distance)
        rospy.logdebug(message)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                               __L A N D _ A C T _ C L I E N T _ F E E D B A C K _ C B
    #
    # Callback function for the feedback of landing action client.
    # ------------------------------------------------------------------------------------------------------------------
    def __land_act_client_feedback_cb(self, feedback):
        message = self.name + ' is landing; actual absolute distance from target height: ' + str(
            feedback.absolute_distance)
        rospy.logdebug(message)


















    # ==================================================================================================================
    #
    #                                   T A K E  O F F  &  L A N D I N G  M E T H O D S
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                             T A K E O F F _ S R V
    #
    # This methods is used to perform a takeoff with service: if there are multiple crazyflies in the scene, they will
    # takeoff once at time.
    # INPUTS:
    #   - height -> z coordinate [m] where the Crazyflie will move during the takeoff;
    # ------------------------------------------------------------------------------------------------------------------
    def takeoff_srv(self, height=DEFAULT_TAKEOFF_HEIGHT):
        # Setting up the takeoff request:
        request = Takeoff_srvRequest()
        request.takeoff_height = height

        # Requesting takeoff:
        self.__takeoff_srv_client(request)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                             T A K E O F F _ A C T N
    #
    # This methods is used to perform a takeoff action: if there are multuple crazyflies in the scene, they will
    # takeoff all (almost) at the same time.
    # INPUTS:
    #   - height -> z coordinate [m] where the Crazyflie will move during the takeoff;
    # ------------------------------------------------------------------------------------------------------------------
    def takeoff_actn(self, height=DEFAULT_TAKEOFF_HEIGHT):
        # Setting up takeoff request:
        goal = TakeoffGoal()
        goal.takeoff_height = height

        # Action requesting;
        self.__takeoff_act_client.send_goal(goal, feedback_cb=self.__takeoff_act_client_feedback_cb)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                                L A N D _ S R V
    #
    # This methods is used to perform a landing service: if there are multuple crazyflies in the scene, they will
    # land once at time.
    # INPUTS:
    #   - height -> z coordinate [m] where the Crazyflie will move and stops its motors;
    # ------------------------------------------------------------------------------------------------------------------
    def land_srv(self, height=DEFAULT_LAND_HEIGHT):
        # Setting up the landing request:
        request = Takeoff_srvRequest()
        request.takeoff_height = height

        # Requesting landing:
        self.__land_srv_client(request)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                                L A N D _ A C T N
    #
    # This methods is used to perform a landing action: if there are multiple crazyflies in the scene, they will
    # land all (almost) at the same time.
    # INPUTS:
    #   - height -> z coordinate [m] where the Crazyflie will move and stops its motors;
    # ------------------------------------------------------------------------------------------------------------------
    def land_actn(self, height=DEFAULT_LAND_HEIGHT):
        # Setting up landing request:
        goal = TakeoffGoal()
        goal.takeoff_height = height

        # Action request:
        self.__land_act_client.send_goal(goal, feedback_cb=self.__land_act_client_feedback_cb)

    # ==================================================================================================================
    #
    #                                   B A S I C  M O V E M E N T S  M E T H O D S
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                               G O _ T O
    #
    # This methods is used to set a destination.
    # INPUTS:
    #   - destination -> Vector3 representing coordinates of destination point [m];
    #   - yaw -> yaw value at destination [deg]
    # ------------------------------------------------------------------------------------------------------------------
    def go_to(self, destination=Vector3(), yaw=0.0):
        if self.status == CfStatus.FLYING:
            self.position_target.desired_position.x = destination.x
            self.position_target.desired_position.y = destination.y
            self.position_target.desired_position.z = destination.z
            self.position_target.desired_yaw = yaw

        else:
            error_message = "Crazyflie " + self.name + " has received a destination, but it's not flying! Please takeoff first!"
            rospy.logerr(error_message)

    def forward(self, distance, velocity=DEFAULT_FORWARD_VELOCITY):
        if self.status == CfStatus.FLYING:
            # Getting actual state:
            actual_state = self.actual_state

            # Computing destination in world frame:
            desitination_goal = Destination3DGoal()
            desitination_goal.destination_info.desired_position.x = actual_state.position.x + math.fabs(distance) * math.cos(actual_state.orientation.yaw)
            desitination_goal.destination_info.desired_position.y = actual_state.position.y + math.fabs(distance) * math.sin(actual_state.orientation.yaw)
            desitination_goal.destination_info.desired_position.z = actual_state.position.z

            desitination_goal.destination_info.desired_yaw = rad2deg(actual_state.orientation.yaw)

            desitination_goal.destination_info.desired_velocity.x = velocity * math.cos(actual_state.orientation.yaw)
            desitination_goal.destination_info.desired_velocity.y = velocity * math.sin(actual_state.orientation.yaw)
            desitination_goal.destination_info.desired_velocity.z = 0.0

            # Calculatinge theorical time to travel to destination:
            desitination_goal.time_duration = distance / velocity

            # Requesting action:
            self.__velocity_3D_motion_act_client.send_goal(desitination_goal,
                                                           feedback_cb=self.__velocity_3D_motion_act_client_feedback_cb)

    def backward(self, distance, velocity=DEFAULT_BACKWARD_VELOCITY):
        pass

    def up(self, distance, velocity):
        pass

    def down(self, distance, velocity):
        pass

    def turn_right(self, angle):
        if self.status == CfStatus.FLYING:
            # Getting actual state:
            actual_state = self.actual_state

            # Computing destination in world frame:
            desitination_goal = Destination3DGoal()
            desitination_goal.destination_info.desired_position.x = actual_state.position.x
            desitination_goal.destination_info.desired_position.y = actual_state.position.y
            desitination_goal.destination_info.desired_position.z = actual_state.position.z

            desitination_goal.destination_info.desired_yaw = rad2deg(actual_state.orientation.yaw) - math.fabs(angle)

            desitination_goal.destination_info.desired_velocity.x = 0.0
            desitination_goal.destination_info.desired_velocity.y = 0.0
            desitination_goal.destination_info.desired_velocity.z = 0.0

            # Calculatinge theorical time to travel to destination:
            desitination_goal.time_duration = 5.0

            # Requesting action:
            self.__velocity_3D_motion_act_client.send_goal(desitination_goal,
                                                           feedback_cb=self.__velocity_3D_motion_act_client_feedback_cb)

    def turn_left(self, angle):
        if self.status == CfStatus.FLYING:
            # Getting actual state:
            actual_state = self.actual_state

            # Setting up new position:
            self.position_target.desired_position.x = actual_state.position.x
            self.position_target.desired_position.y = actual_state.position.y
            self.position_target.desired_position.z = actual_state.position.z
            self.position_target.desired_yaw = rad2deg(actual_state.orientation.yaw) + math.fabs(angle)
        else:
            error_msg = "Crazyflie " + self.name + " has received a command, but it's not flying! Please takeoff first!"
            rospy.logerr(error_msg)