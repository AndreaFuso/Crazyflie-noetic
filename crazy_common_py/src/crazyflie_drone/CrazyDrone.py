# ROS
import rospy
import actionlib

# Generic modules
import logging
import time

# Custom modules
from crazy_common_py.dataTypes import Vector3
from crazy_common_py.default_topics import DEFAULT_TAKEOFF_ACT_TOPIC, DEFAULT_LAND_ACT_TOPIC, DEFAULT_REL_VEL_TOPIC, \
    DEFAULT_REL_POS_TOPIC, DEFAULT_CF_STATE_TOPIC, DEFAULT_100Hz_PACE_TOPIC, DEFAULT_MOTOR_CMD_TOPIC, \
    DEFAULT_ACTUAL_DESTINATION_TOPIC
from crazy_common_py.common_functions import deg2rad
from crazy_common_py.constants import DEFAULT_TAKEOFF_HEIGHT
# Action messages:
from crazyflie_messages.msg import TakeoffAction, TakeoffGoal, TakeoffResult, TakeoffFeedback
from crazyflie_messages.msg import Destination3DAction, Destination3DGoal, Destination3DFeedback, Destination3DResult
from crazyflie_messages.msg import CrazyflieState, Attitude, Position
from std_msgs.msg import Empty

# Crazyflie API
import cflib.crtp
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig

class CrazyDrone:
    # ==================================================================================================================
    #
    #                                               C O N S T R U C T O R
    #
    # This class completely handle one real Crazyflie.
    # INPUTS:
    #   1) URI -> URI of the Crazyflie;
    #   2) initialPosition -> Vector3 object with the inital position coordinates;
    # ==================================================================================================================
    def __init__(self, name, URI, initialPosition):
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                           P R O P E R T I E S  I N I T I A L I Z A T I O N
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Variable to understand when initial operations are ended (otherwise problem with 100Hz subscriber):
        self.__initialOperationsEnded = False

        # Name (used for setting up topics, actions and services):
        self.cfName = name

        # URI of the crazyflie (unique address):
        self.URI = URI

        # Initial position (Vector3):
        self.__initial_position = initialPosition

        '''
            Logger can contain maximum 26 bytes:
                - LOG_FLOAT -> float => 4 bytes
        '''
        # Attitude logger configuration (6 floats => 24/26 bytes):
        self.__attitude_logger_config = LogConfig(name='attitude_conf', period_in_ms=10)
        self.__attitude_logger_config.add_variable('stabilizer.roll', 'float')
        self.__attitude_logger_config.add_variable('stabilizer.pitch', 'float')
        self.__attitude_logger_config.add_variable('stabilizer.yaw', 'float')
        self.__attitude_logger_config.add_variable('gyro.x', 'float')
        self.__attitude_logger_config.add_variable('gyro.y', 'float')
        self.__attitude_logger_config.add_variable('gyro.z', 'float')

        # State logger configuration (6 floats => 24/26 bytes):
        self.__state_logger_config = LogConfig(name='state_conf', period_in_ms=10)
        self.__state_logger_config.add_variable('stateEstimate.x', 'float')
        self.__state_logger_config.add_variable('stateEstimate.y', 'float')
        self.__state_logger_config.add_variable('stateEstimate.z', 'float')
        self.__state_logger_config.add_variable('stateEstimate.vx', 'float')
        self.__state_logger_config.add_variable('stateEstimate.vy', 'float')
        self.__state_logger_config.add_variable('stateEstimate.vz', 'float')

        # Reference velocity state logger configuration (4 floats => 16/26 bytes):
        self.__desired_state_logger_config = LogConfig(name='desired_state_config', period_in_ms=10)
        self.__desired_state_logger_config.add_variable('posCtl.targetVX', 'float')
        self.__desired_state_logger_config.add_variable('posCtl.targetVY', 'float')
        self.__desired_state_logger_config.add_variable('posCtl.targetVZ', 'float')
        self.__desired_state_logger_config.add_variable('controller.yawRate', 'float')

        # Controller output logger configuration (4 floats => 16/26 bytes):
        self.__controller_output_config = LogConfig(name='controller_output_conf', period_in_ms=10)
        self.__controller_output_config.add_variable('controller.cmd_thrust')
        self.__controller_output_config.add_variable('controller.cmd_roll')
        self.__controller_output_config.add_variable('controller.cmd_pitch')
        self.__controller_output_config.add_variable('controller.cmd_yaw')

        # State:
        self.__state = CrazyflieState()

        # Controller output:
        self.__controller_output = Attitude()

        # Desired position:
        self.__desired_state = Position()

        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       S U B S C R I B E R S  S E T U P
        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        self.__pace_100Hz_sub = rospy.Subscriber('/' + DEFAULT_100Hz_PACE_TOPIC, Empty, self.__pace_100Hz_cb)


        # Subscriber to read the desired velocity computed by the MPC controller
        self.mpc_velocity_sub = rospy.Subscriber('/' + self.cfName + '/mpc_velocity',
                                                          Position, self.__mpc_velocity_callback)
        self.mpc_velocity = Position()
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       P U B L I S H E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Publisher used to publish real Crazyflie state:
        self.__state_pub = rospy.Publisher('/' + self.cfName + '/' + DEFAULT_CF_STATE_TOPIC, CrazyflieState,
                                           queue_size=1)

        # Publisher to publish motor command for real crazyflie:
        self.__motor_command_pub = rospy.Publisher('/' + self.cfName + '/' + DEFAULT_MOTOR_CMD_TOPIC, Attitude,
                                                   queue_size=1)

        # Publisher to publish the reference:
        self.__desired_state_pub = rospy.Publisher('/' + self.cfName + '/' + DEFAULT_ACTUAL_DESTINATION_TOPIC, Position,
                                                   queue_size=1)

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                           S E R V I C E S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                           A C T I O N S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Takeoff action:
        self.__takeoff_act = actionlib.SimpleActionServer('/' + name + '/' + DEFAULT_TAKEOFF_ACT_TOPIC,
                                                          TakeoffAction, self.__takeoff_act_callback, False)
        self.__takeoff_act.start()

        # Landing action:
        self.__land_act = actionlib.SimpleActionServer('/' + name + '/' + DEFAULT_LAND_ACT_TOPIC,
                                                       TakeoffAction, self.__land_act_callback, False)
        self.__land_act.start()

        # Relative velocity motion:
        self.__rel_vel_move_act = actionlib.SimpleActionServer('/' + name + '/' + DEFAULT_REL_VEL_TOPIC,
                                                               Destination3DAction,
                                                               self.__rel_vel_move_act_callback, False)
        self.__rel_vel_move_act.start()

        self.__rel_vel_move_client = actionlib.SimpleActionClient('__rel_vel_move_client', Destination3DAction)

        # Waits until the action server has started up and started listening for goals.
        self.__rel_vel_move_client.wait_for_server()

        # Creates a goal to send to the action server.
        self.goal = Destination3DAction()








        # Relative displacement motion;
        self.__rel_displ_move_act = actionlib.SimpleActionServer('/' + name + '/' + DEFAULT_REL_POS_TOPIC,
                                                                 Destination3DAction,
                                                                 self.__rel_displ_move_act_callback, False)
        self.__rel_displ_move_act.start()

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                        I N I T I A L  O P E R A T I O N S
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Drivers initialization:
        cflib.crtp.init_drivers()

        # Instantiation of SyncCrazyflie and opening communication:
        self.__scf = SyncCrazyflie(URI)
        self.__scf.open_link()

        # Instantiation of MotionCommander:
        self.__mc = MotionCommander(self.__scf)

        # Attitude logger:
        self.__attitude_logger = SyncLogger(self.__scf, self.__attitude_logger_config)
        self.__attitude_logger.connect()

        # State logger:
        self.__state_logger = SyncLogger(self.__scf, self.__state_logger_config)
        self.__state_logger.connect()

        # Controller output logger:
        self.__controller_output_logger = SyncLogger(self.__scf, self.__controller_output_config)
        self.__controller_output_logger.connect()

        # Desired state logger:
        self.__desired_state_logger = SyncLogger(self.__scf, self.__desired_state_logger_config)
        self.__desired_state_logger.connect()

        self.__initialOperationsEnded = True
        #self.__mc.take_off()
        #self.__scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 20000)

    # ==================================================================================================================
    #
    #                                     C A L L B A C K  M E T H O D S  (T O P I C S)
    #
    # ==================================================================================================================
    def __pace_100Hz_cb(self, msg):
        if self.__initialOperationsEnded:
            # Getting data from loggers:
            attitude_data = self.__attitude_logger.next()
            state_data = self.__state_logger.next()
            controller_output_data = self.__controller_output_logger.next()
            desired_state_data = self.__desired_state_logger.next()

            # Extracting attitude:
            self.__state.orientation.roll = deg2rad(attitude_data[1]['stabilizer.roll'])
            self.__state.orientation.pitch = deg2rad(- attitude_data[1]['stabilizer.pitch'])
            self.__state.orientation.yaw = deg2rad(attitude_data[1]['stabilizer.yaw'])
            self.__state.rotating_speed.x = deg2rad(attitude_data[1]['gyro.x'])
            self.__state.rotating_speed.y = deg2rad(attitude_data[1]['gyro.y'])
            self.__state.rotating_speed.z = deg2rad(attitude_data[1]['gyro.z'])

            # Extracting state:
            self.__state.position.x = state_data[1]['stateEstimate.x']
            self.__state.position.y = state_data[1]['stateEstimate.y']
            self.__state.position.z = state_data[1]['stateEstimate.z']
            self.__state.velocity.x = state_data[1]['stateEstimate.vx']
            self.__state.velocity.y = state_data[1]['stateEstimate.vy']
            self.__state.velocity.z = state_data[1]['stateEstimate.vz']

            # Extracting controller output:
            self.__controller_output.desired_attitude.roll = controller_output_data[1]['controller.cmd_roll']
            self.__controller_output.desired_attitude.pitch = controller_output_data[1]['controller.cmd_pitch']
            self.__controller_output.desired_attitude.yaw = controller_output_data[1]['controller.cmd_yaw']
            self.__controller_output.desired_thrust = controller_output_data[1]['controller.cmd_thrust']

            # Extracting reference data:
            self.__desired_state.desired_velocity.x = desired_state_data[1]['posCtl.targetVX']
            self.__desired_state.desired_velocity.y = desired_state_data[1]['posCtl.targetVY']
            self.__desired_state.desired_velocity.z = desired_state_data[1]['posCtl.targetVZ']
            self.__desired_state.desired_yaw_rate = desired_state_data[1]['controller.yawRate']

    
            # Publishing the state:
            self.__state_pub.publish(self.__state)

            # Publishing the motor command:
            self.__motor_command_pub.publish(self.__controller_output)

            # Publishing desired:
            self.__desired_state_pub.publish(self.__desired_state)

            #print(data)
            #print('ROLL: ', data[1]['stabilizer.roll'], '; PITCH: ', data[1]['stabilizer.pitch'], '; YAW: ', data[1]['stabilizer.yaw'])



    # ------------------------------------------------------------------------------------------------------------------
    #
    #                               __M P C _ V E L O C I T Y _ C A L L B A C K
    #
    # This callback gets the desired velocity computed by the mpc controller and sets the velocity target so that
    # the velocity command is provided to the velocity controller
    # ------------------------------------------------------------------------------------------------------------------
    def __mpc_velocity_callback(self, msg):
        
        # self.position_target.desired_velocity.x = msg.desired_velocity.x
        # self.position_target.desired_velocity.y = msg.desired_velocity.y
        # self.position_target.desired_velocity.z = msg.desired_velocity.z

        self.goal.destination_info.desired_velocity.x = msg.desired_velocity.x
        self.goal.destination_info.desired_velocity.y = msg.desired_velocity.y
        self.goal.destination_info.desired_velocity.z = msg.desired_velocity.z


        # Sends the goal to the action server.
        self.__rel_vel_move_client.send_goal(self.goal)

        # Waits for the server to finish performing the action.
        self.__rel_vel_move_client.wait_for_result()




    # ==================================================================================================================
    #
    #                                 C A L L B A C K  M E T H O D S  (S E R V I C E S)
    #
    # ==================================================================================================================

    # ==================================================================================================================
    #
    #                                 C A L L B A C K  M E T H O D S  (A C T I O N S)
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                   T A K E O F F _ A C T _ C A L L B A C K
    #
    # Callback for takeoff action.
    # ------------------------------------------------------------------------------------------------------------------
    def __takeoff_act_callback(self, goal):
        if goal.takeoff_height <= 0:
            desired_takeoff_height = DEFAULT_TAKEOFF_HEIGHT
        else:
            desired_takeoff_height = goal.takeoff_height
        self.__mc.take_off(height=desired_takeoff_height)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                T A K E O F F _ A C T _ F E E D B A C K _ C B
    #
    # Feedback callback method for takeoff action.
    # ------------------------------------------------------------------------------------------------------------------
    def __takeoff_act_feedback_cb(self, feedback):
        pass

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                   L A N D _ A C T _ C A L L B A C K
    #
    # Callback for land action.
    # ------------------------------------------------------------------------------------------------------------------
    def __land_act_callback(self, goal):
        self.__mc.land()

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                L A N D _ A C T _ F E E D B A C K _ C B
    #
    # Feedback callback method for land action.
    # ------------------------------------------------------------------------------------------------------------------
    def __land_act_feedback_cb(self, feedback):
        pass

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                   R E L _ V E L _ M O V E _ A C T _ C A L L B A C K
    #
    # Callback for relative velocity motion action.
    # ------------------------------------------------------------------------------------------------------------------
    def __rel_vel_move_act_callback(self, goal):

        success = False

        # Getting request parameters;
        vx_des = goal.destination_info.desired_velocity.x
        vy_des = goal.destination_info.desired_velocity.y
        vz_des = goal.destination_info.desired_velocity.z
        yaw_rate_des = - goal.destination_info.desired_yaw_rate

        time_duration = goal.time_duration

        if time_duration <= 0:
            time_duration = 1.0

        # Getting initial time and time duration (used for fixed time duration request):
        t0 = rospy.get_rostime()
        ros_time_duration = rospy.Duration.from_sec(time_duration)
        final_rostime = t0 + ros_time_duration

        feedback = Destination3DFeedback()

        # Starting the motion:
        self.__mc.start_linear_motion(vx_des, vy_des, vz_des, yaw_rate_des)

        # Wait time duration:
        while True:
            # Getting current time:
            actual_time = rospy.get_rostime()

            # Remaining time:
            remaining_time = (final_rostime - actual_time).to_sec()

            # Publishing absolute distance as feedback:
            feedback.remaining_time = remaining_time
            self.__rel_vel_move_act.publish_feedback(feedback)

            # If time duration has elapsed exit the cycle:
            if remaining_time <= 0:
                success = True
                break

        # Once time duration elapsed let's stop the crazyflie:
        self.__mc.stop()

        # Result:
        response = Destination3DResult()
        response.result = success
        self.__rel_vel_move_act.set_succeeded(response)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                              R E L _ V E L _ M O V E _ A C T _ F E E D B A C K _ C B
    #
    # Feedback callback method for relative velocity motion action.
    # ------------------------------------------------------------------------------------------------------------------
    def __rel_vel_move_act_feedback_cb(self, feedback):
        pass

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                   R E L _ D I S P L _ M O V E _ A C T _ C A L L B A C K
    #
    # Callback for relative displacement motion action.
    # ------------------------------------------------------------------------------------------------------------------
    def __rel_displ_move_act_callback(self, goal):
        pass

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                              R E L _ D I S P L _ M O V E _ A C T _ F E E D B A C K _ C B
    #
    # Feedback callback method for relative displacement motion action.
    # ------------------------------------------------------------------------------------------------------------------
    def __rel_displ_move_act_feedback_cb(self, feedback):
        pass

    # ==================================================================================================================
    #
    #                                               E X I T  M E T H O D S
    #
    # All operations to perform if an error occurs (like KeyboardInterrupt to stop the execution).
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                           E X I T _ O P E R A T I O N S
    #
    # This method performs all the required exiting operations.
    # ------------------------------------------------------------------------------------------------------------------
    def exit_operations(self):
        # Land the drone:
        self.__mc.land()

        # Stop logger:
        self.__attitude_logger.disconnect()

        # Closing communication with the crazyflie:
        self.__scf.close_link()

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                           __ E X I T __
    #
    # This method is called when an error occurs, performing some operations before destroy class instance.
    # ------------------------------------------------------------------------------------------------------------------
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.exit_operations()


