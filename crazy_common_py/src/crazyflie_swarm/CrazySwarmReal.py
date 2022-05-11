# ROS MODULES
from time import sleep
import rospy
import actionlib

# Action
from crazyflie_messages.msg import TakeoffAction, TakeoffGoal, TakeoffResult, TakeoffFeedback
from crazyflie_messages.msg import EmptyAction, EmptyGoal, EmptyResult, EmptyFeedback
from crazy_common_py.default_topics import DEFAULT_FLOCK_TOPIC, DEFAULT_CF_STATE_TOPIC, \
     DEFAULT_100Hz_PACE_TOPIC, DEFAULT_MOTOR_CMD_TOPIC, DEFAULT_ACTUAL_DESTINATION_TOPIC
from crazy_common_py.constants import DEFAULT_LEADER
from std_msgs.msg import Empty
from crazyflie_messages.msg import Position, CrazyflieState, SwarmStates, Attitude, \
                                   SwarmControllerOutputs, SwarmDesiredStates

# Custom modules
from crazy_common_py.common_functions import deg2rad, extractCfNumber
from crazy_common_py.constants import *

# Crazyflie API
import cflib.crtp
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.commander import Commander
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import Swarm, CachedCfFactory, _Factory

# Other imports
from collections import namedtuple


StateEstimateLog = namedtuple('StateEstimateLog', 'x y z vx vy vz')
AttitudeLog = namedtuple('AttitudeLog', 'roll pitch yaw gyro_x gyro_y gyro_z')
RefVelocityLog = namedtuple('RefVelocityLog', 'target_vx target_vy target_vz yawrate')
ControllerOutputLog= namedtuple('ControllerOutputLog', 'cmd_thrust cmd_roll cmd_pitch cmd_yaw')

class CrazySwarmReal:
    # ==================================================================================================================
    #
    #              C O N S T R U C T O R
    #
    # This class completely handle one virtual swarm of crazyflies.
    # INPUTS:
    #   1) cf_names -> list of names of each Crazyflie within the virtual swarm;
    #
    # ==================================================================================================================
    def __init__(self, cf_names):
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #              P R O P E R T I E S  I N I T I A L I Z A T I O N
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        # Variable to understand when initial operations are ended 
        # (otherwise problem with 100Hz subscriber):
        self.__initialOperationsEnded = False

        # List of crazyflies names:
        self.cf_names = cf_names

        # Number of crazyflies:
        self.number_of_cfs = len(self.cf_names)

        # Dictionaries for commanders initialization
        self.mc_dict = dict()
        self.c_dict = dict()

        # State logger configuration (6 floats => 24/26 bytes):
        self.__state_logger_config = LogConfig(name='state_conf', period_in_ms=10)
        self.__state_logger_config.add_variable('stateEstimate.x', 'float')
        self.__state_logger_config.add_variable('stateEstimate.y', 'float')
        self.__state_logger_config.add_variable('stateEstimate.z', 'float')
        self.__state_logger_config.add_variable('stateEstimate.vx', 'float')
        self.__state_logger_config.add_variable('stateEstimate.vy', 'float')
        self.__state_logger_config.add_variable('stateEstimate.vz', 'float')

        # Attitude logger configuration (6 floats => 24/26 bytes):
        self.__attitude_logger_config = LogConfig(name='attitude_conf', 
                                                            period_in_ms=10)
        self.__attitude_logger_config.add_variable('stabilizer.roll', 'float')
        self.__attitude_logger_config.add_variable('stabilizer.pitch', 'float')
        self.__attitude_logger_config.add_variable('stabilizer.yaw', 'float')
        self.__attitude_logger_config.add_variable('gyro.x', 'float')
        self.__attitude_logger_config.add_variable('gyro.y', 'float')
        self.__attitude_logger_config.add_variable('gyro.z', 'float')

         # Controller output logger configuration (4 floats => 16/26 bytes):
        self.__controller_output_config = LogConfig(name='controller_output_conf', 
                                                                    period_in_ms=10)
        self.__controller_output_config.add_variable('controller.cmd_thrust', 'float')
        self.__controller_output_config.add_variable('controller.cmd_roll', 'float')
        self.__controller_output_config.add_variable('controller.cmd_pitch', 'float')
        self.__controller_output_config.add_variable('controller.cmd_yaw', 'float')

        # Reference velocity state logger configuration (4 floats => 16/26 bytes):
        self.__desired_state_logger_config = LogConfig(name='desired_state_config', 
                                                                    period_in_ms=10)
        self.__desired_state_logger_config.add_variable('posCtl.targetVX', 'float')
        self.__desired_state_logger_config.add_variable('posCtl.targetVY', 'float')
        self.__desired_state_logger_config.add_variable('posCtl.targetVZ', 'float')
        self.__desired_state_logger_config.add_variable('controller.yawRate', 'float')

        # Dictionaries for logger data
        self.attitude_data_dict = dict()
        self.state_data_dict = dict()
        self.controller_output_data_dict = dict()
        self.desired_state_data_dict = dict()


        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                 S U B S C R I B E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Subscriber to pace 100Hz:
        self.pace_100Hz_sub = rospy.Subscriber('/' + DEFAULT_100Hz_PACE_TOPIC, 
                                            Empty, self.__pace_100Hz_sub_callback)

        # List of state subscribers:
        self.state_subs = []
        self.states = []
        self.__make_state_subs()
        

        # List of mpc velocity subscribers:
        self.mpc_velocity_subs = []
        self.mpc_velocities = []
        self.desired_vx = dict()
        self.desired_vy = dict()
        self.desired_vz = dict()
        self.__make_mpc_velocity_subs()
        
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                 P U B L I S H E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # States publisher:
        self.states_pub = rospy.Publisher('/swarm/states', SwarmStates, queue_size=1)

        # List of controller outputs
        self.controller_outputs = []
        self.__make_controller_outputs_publishers()

        # List of desired states
        self.desired_states = []
        self.__make_desired_states_publishers()

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                   S E R V I C E S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                    A C T I O N S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Action to make the entire swarm take off :
        self.__swarm_takeoff_act = actionlib.SimpleActionServer('/swarm/takeoff_actn',
                            TakeoffAction, self.__swarm_takeoff_act_callback, False)
        self.__swarm_takeoff_act.start()


        # Action to make the entire swarm land:
        self.__swarm_land_act = actionlib.SimpleActionServer('/swarm/land_actn', 
                                 TakeoffAction, self.__swarm_land_act_callback, False)
        self.__swarm_land_act.start()

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #             I N I T I A L  O P E R A T I O N S
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        # Drivers initialization:
        cflib.crtp.init_drivers()

        # List of URIs
        self.uris = self.create_uris_list(cf_names)

        print(self.uris)

        # Instantiation of Swarm and opening communication:
        self.__swarm = Swarm(self.uris, factory = _Factory())
        self.__swarm.open_links()

        # Loggers configuration for all the agents

        self.__attitude_loggers = dict()
        self.__state_loggers = dict()
        self.__controller_output_loggers = dict()
        self.__desired_state_loggers = dict()




        # Configuring the loggers
        self.state_loggers_swarm()
        self.attitude_loggers_swarm()
        self.controller_output_loggers_swarm()
        self.desired_state_loggers_swarm()

        # Create commanders for the swarm
        self.create_commanders_dict_swarm()



        # Ending initial operations
        self.__initialOperationsEnded = True





    # ==================================================================================================================
    #
    #         I N I T I A L  O P E R A T I O N S  M E T H O D S
    #
    # ==================================================================================================================

    #+++++++++++++++++++ STATE SUBSCRIBER LIST METHOD ++++++++++++++++++++++++++++++

    def __make_state_subs(self):
        for cf_name in self.cf_names:
            # Subscribers to read the state of the drones
            tmp_sub = rospy.Subscriber('/' + cf_name + '/' + 
                        DEFAULT_CF_STATE_TOPIC, CrazyflieState, self.__state_cb)
            self.state_subs.append(tmp_sub)
            self.states.append(CrazyflieState())

    #+++++++++++++++++++ MPC VELOCITY SUBSCRIBER LIST METHOD ++++++++++++++++++++++++

    def __make_mpc_velocity_subs(self):
        for cf_name in self.cf_names:
            # Subscribers to read the desired velocity computed by the MPC controller
            tmp_sub = rospy.Subscriber('/' + cf_name + 
                        '/mpc_velocity', Position, self.__mpc_velocity_callback)
            self.mpc_velocity_subs.append(tmp_sub)
            self.mpc_velocities.append(Position())

    #+++++++++++++++++++ C.O. PUBLISHERS LIST METHOD +++++++++++++++++++++++++++++++

    def __make_controller_outputs_publishers(self):
        self.controller_outputs_pubs = []
        for cf_name in self.cf_names:
            tmp_pub = rospy.Publisher('/' + cf_name + '/' + 
                            DEFAULT_MOTOR_CMD_TOPIC, Attitude, queue_size=1)
            self.controller_outputs_pubs.append(tmp_pub)
            self.controller_outputs.append(Attitude())

    #+++++++++++++++++++ C.O. PUBLISHERS PUBLISH METHOD ++++++++++++++++++++++++++++

    def __controller_outputs_pub(self, controller_outputs):
        index = 0
        for index, controller_output_pub in enumerate(self.controller_outputs_pubs):
            controller_output_pub.publish(controller_outputs.controller_outputs[index])
            # print('co pub is ok')

    #+++++++++++++++++++ D.S. PUBLISHERS LIST METHOD +++++++++++++++++++++++++++++++

    def __make_desired_states_publishers(self):
        self.desired_states_pubs = []
        for cf_name in self.cf_names:
            tmp_pub = rospy.Publisher('/' + cf_name + '/' + 
                            DEFAULT_ACTUAL_DESTINATION_TOPIC, Position, queue_size=1)
            self.desired_states_pubs.append(tmp_pub)
            self.desired_states.append(Position())

    #+++++++++++++++++++ D.S. PUBLISHERS PUBLISH METHOD ++++++++++++++++++++++++++++

    def __desired_states_pub(self, desired_states):
        index = 0
        for index, desired_state_pub in enumerate(self.desired_states_pubs):
            desired_state_pub.publish(desired_states.desired_states[index])
            # print('ds pub is ok')

    #++++++++++++++++++++++++ CREATE LIST OF URIS +++++++++++++++++++++++++++++++++

    def create_uris_list(self, cf_names):
        uris = []
        for name in cf_names:
            num_ID = int(name[2:]) - 1
            uri = 'radio://0/80/2M/E7E7E7E7E' + hex(num_ID)[-1]
            uris.append(uri)
        
        return uris

    #++++++++ MOTION COMMANDER AND COMMANDER INSTANTIATION METHOD +++++++++++++++++

    # Creating a list of motion commanders to be used for the single drones
    # and for the swarm

    def create_commanders_dict_drone(self, scf):

        # Motion commander instance
        motion_commander = MotionCommander(scf)        
        self.mc_dict[scf._link_uri] = motion_commander

        # Commander instance to send control setpoints
        commander = Commander(scf.cf)
        commander.set_client_xmode(enabled=True)

        self.c_dict[scf._link_uri] = commander

    def create_commanders_dict_swarm(self):
        self.__swarm.parallel_safe(self.create_commanders_dict_drone)

    # ==================================================================================================================
    #
    #                                     C A L L B A C K  M E T H O D S  (T O P I C S)
    #
    # ==================================================================================================================
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

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                  __P A C E _ 1 0 0 H Z _ S U B _ C A L L B A C K
    #
    # This callback is called whenever an Empty message is published by pace_100Hz_node; it is used to develop the main
    # routine.
    # ------------------------------------------------------------------------------------------------------------------
    def __pace_100Hz_sub_callback(self, msg):

        if self.__initialOperationsEnded:

            # Initializing states message:
            states = SwarmStates()
            controller_outputs = SwarmControllerOutputs()
            desired_states = SwarmDesiredStates()



            cf_index = 0

            for uri in self.uris:

                # Extracting information from the loggers
                self.state_data_dict[uri] = self.__state_loggers[uri].next()
                self.attitude_data_dict[uri] = self.__attitude_loggers[uri].next()
                self.controller_output_data_dict[uri] = \
                                    self.__controller_output_loggers[uri].next()
                self.desired_state_data_dict[uri] = \
                                    self.__desired_state_loggers[uri].next()


                # States
                self.states[cf_index].name = 'cf' + str(cf_index + 1)
                self.states[cf_index].position.x = \
                    self.state_data_dict[uri][1]['stateEstimate.x']
                self.states[cf_index].position.y = \
                    self.state_data_dict[uri][1]['stateEstimate.y']
                self.states[cf_index].position.z = \
                    self.state_data_dict[uri][1]['stateEstimate.z']
                self.states[cf_index].velocity.x = \
                    self.state_data_dict[uri][1]['stateEstimate.vx']
                self.states[cf_index].velocity.y = \
                    self.state_data_dict[uri][1]['stateEstimate.vy']
                self.states[cf_index].velocity.z = \
                    self.state_data_dict[uri][1]['stateEstimate.vz']

                # Attitudes
                self.states[cf_index].orientation.roll = \
                    deg2rad(self.attitude_data_dict[uri][1]['stabilizer.roll'])
                self.states[cf_index].orientation.pitch = \
                    deg2rad(-self.attitude_data_dict[uri][1]['stabilizer.pitch'])
                self.states[cf_index].orientation.yaw = \
                    deg2rad(self.attitude_data_dict[uri][1]['stabilizer.yaw'])
                self.states[cf_index].rotating_speed.x = \
                    deg2rad(self.attitude_data_dict[uri][1]['gyro.x'])
                self.states[cf_index].rotating_speed.y = \
                    deg2rad(self.attitude_data_dict[uri][1]['gyro.y'])
                self.states[cf_index].rotating_speed.z = \
                    deg2rad(self.attitude_data_dict[uri][1]['gyro.z'])
                
                # Controller outputs
                self.controller_outputs[cf_index].desired_thrust = \
                    self.controller_output_data_dict[uri][1]['controller.cmd_thrust']
                self.controller_outputs[cf_index].desired_attitude.roll = \
                    self.controller_output_data_dict[uri][1]['controller.cmd_roll']
                self.controller_outputs[cf_index].desired_attitude.pitch = \
                    self.controller_output_data_dict[uri][1]['controller.cmd_pitch']
                self.controller_outputs[cf_index].desired_attitude.yaw = \
                    self.controller_output_data_dict[uri][1]['controller.cmd_yaw']

                # Desired states
                self.desired_states[cf_index].desired_velocity.x = \
                    self.desired_state_data_dict[uri][1]['posCtl.targetVX']
                self.desired_states[cf_index].desired_velocity.y = \
                    self.desired_state_data_dict[uri][1]['posCtl.targetVY']
                self.desired_states[cf_index].desired_velocity.z = \
                    self.desired_state_data_dict[uri][1]['posCtl.targetVZ']
                self.desired_states[cf_index].desired_yaw_rate = \
                    self.desired_state_data_dict[uri][1]['controller.yawRate']
                
                cf_index += 1

            states.states = self.states
            controller_outputs.controller_outputs = self.controller_outputs
            desired_states.desired_states = self.desired_states

            print(self.states)
            print(self.controller_outputs)
            # print(self.desired_states)

            self.states_pub.publish(states)
            self.__controller_outputs_pub(controller_outputs)
            self.__desired_states_pub(desired_states)
            


    # ------------------------------------------------------------------------------------------------------------------
    #
    #         __M P C _ V E L O C I T Y _ C A L L B A C K
    #
    # This callback gets the desired velocity computed by the mpc controller 
    # and sets the velocity target so that the velocity commands are given as 
    # velocity setpoints to the drones composing the swarm
    # ------------------------------------------------------------------------------------------------------------------
    def __mpc_velocity_callback(self, msg):
        
        cf_name = msg.name
        num_ID = int(cf_name[2:]) - 1
        uri = 'radio://0/80/2M/E7E7E7E7E' + hex(num_ID)[-1]

        self.desired_vx[uri] = msg.desired_velocity.x
        self.desired_vy[uri] = msg.desired_velocity.y
        self.desired_vz[uri] = msg.desired_velocity.z

    # ==================================================================================================================
    #
    #            C A L L B A C K  M E T H O D S  (A C T I O N S)
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #         __S W A R M _ T A K E O F F _ A C T _ C A L L B A C K
    #
    # This method is used as the swarm takeoff action.
    # ------------------------------------------------------------------------------------------------------------------
    
    def __swarm_takeoff_act_callback(self, goal):
        # Output:
        result = TakeoffResult()
        
        self.takeoff_swarm()

        self.__swarm_takeoff_act.set_succeeded(result)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #         __S W A R M _ L A N D _ A C T _ C A L L B A C K
    #
    # This method is used as the swarm land action.
    # ------------------------------------------------------------------------------------------------------------------
    def __swarm_land_act_callback(self, goal):
        # Output:
        result = TakeoffResult()
        
        self.land_swarm()
        
        self.__swarm_land_act.set_succeeded(result)
    
    # ==================================================================================================================
    #
    #         F E E D B A C K  C A L L B A C K  M E T H O D S  (A C T I O N S)
    #
    # ==================================================================================================================
    def __cf_takeoff_feedback_cb(self, feedback):
        pass


    # ==================================================================================================================
    #
    #        L O G G E R S     C O N F I G U R A T I O N    M E T H O D S
    #
    # ==================================================================================================================

    # Dictionaries are used to store the SyncLogger instances and extract data 
    # within the 100 Hz callback
    # The key used to identify the drones is the URI string


    #++++++++++++++++++++++++ CREATE STATE LOGGERS +++++++++++++++++++++++++++++++++

    def state_logger_drone(self, scf):

        # SyncLogger instantiation
        self.__state_logger = SyncLogger(scf, self.__state_logger_config)
        self.__state_logger.connect()
        # Storing SyncLogger instance inside the dictionary
        self.__state_loggers[scf._link_uri] = self.__state_logger

    def state_loggers_swarm(self):
        """
        Configuring the loggers to be used 
        """
        self.__swarm.parallel_safe(self.state_logger_drone)


    #++++++++++++++++++++++++ CREATE ATTITUDE LOGGERS ++++++++++++++++++++++++++++++

    def attitude_logger_drone(self, scf):

        # SyncLogger instantiation
        self.__attitude_logger = SyncLogger(scf, self.__attitude_logger_config)
        self.__attitude_logger.connect()
        # Storing SyncLogger instance inside the dictionary
        self.__attitude_loggers[scf._link_uri] = self.__attitude_logger

    def attitude_loggers_swarm(self):
        """
        Configuring the loggers to be used 
        """
        self.__swarm.parallel_safe(self.attitude_logger_drone)

    #++++++++++++++++++++ CREATE CONTROLLER OUTPUT LOGGERS +++++++++++++++++++++++++

    def controller_output_logger_drone(self, scf):

        # SyncLogger instantiation
        self.__controller_output_logger = SyncLogger(scf, 
                                    self.__controller_output_config)
        self.__controller_output_logger.connect()
        # Storing SyncLogger instance inside the dictionary
        self.__controller_output_loggers[scf._link_uri] = \
                                    self.__controller_output_logger

    def controller_output_loggers_swarm(self):
        """
        Configuring the loggers to be used 
        """
        self.__swarm.parallel_safe(self.controller_output_logger_drone)

    #+++++++++++++++++++++ CREATE DESIRED STATE LOGGERS ++++++++++++++++++++++++++++

    def desired_state_logger_drone(self, scf):

        # SyncLogger instantiation
        self.__desired_state_logger = SyncLogger(scf, 
                                    self.__desired_state_logger_config)
        self.__desired_state_logger.connect()
        # Storing SyncLogger instance inside the dictionary
        self.__desired_state_loggers[scf._link_uri] = self.__desired_state_logger

    def desired_state_loggers_swarm(self):
        """
        Configuring the loggers to be used 
        """
        self.__swarm.parallel_safe(self.desired_state_logger_drone)



    # ==================================================================================================================
    #
    #         M E T H O D S    F O R    A C T I O N S    
    #
    # ==================================================================================================================


    #+++++++++++++++++++++++ TAKEOFF METHOD ++++++++++++++++++++++++++++++++++++++

    def takeoff_drone4swarm(self, scf):
        self.mc_dict[scf._link_uri].take_off(height=0.3)

    def takeoff_swarm(self):
        print('takeoff action')
        self.__swarm.parallel_safe(self.takeoff_drone4swarm)

    #+++++++++++++++++++++++++ LAND METHOD +++++++++++++++++++++++++++++++++++++++

    def land_drone4swarm(self, scf):
        self.mc_dict[scf._link_uri].land()

    def land_swarm(self):
        print('land action')
        self.__swarm.parallel_safe(self.land_drone4swarm)

    #+++++++++++++++++++++ VELOCITY SETPOINT METHOD ++++++++++++++++++++++++++++++

    def velocity_setpoint_drone4swarm(self, scf):
        self.c_dict[scf._link_uri].send_velocity_world_setpoint(
                                        self.desired_vx[scf._link_uri], 
                                        self.desired_vy[scf._link_uri],
                                        self.desired_vz[scf._link_uri])

    def velocity_setpoint_swarm(self):
        print('... sending velocity setpoints')
        self.__swarm.parallel_safe(self.velocity_setpoint_drone4swarm)

    # ==================================================================================================================
    #
    #                   E X I T  M E T H O D S
    #
    # All operations to perform if an error occurs (like KeyboardInterrupt to stop 
    # the execution).
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                   E X I T _ O P E R A T I O N S
    #
    # This method performs all the required exiting operations.
    # ------------------------------------------------------------------------------------------------------------------
    def exit_operations(self):

        print('...about to land the swarm')
        # Land the swarm
        self.land_swarm()

        print('swarm landed')

        print('...about to close links')
        # Closing communication with the crazyflie:
        self.__swarm.close_links()

        print('links have been closed')

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                   __ E X I T __
    #
    # This method is called when an error occurs, performing some operations before 
    # destroying class instance.
    # ------------------------------------------------------------------------------------------------------------------
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.exit_operations()