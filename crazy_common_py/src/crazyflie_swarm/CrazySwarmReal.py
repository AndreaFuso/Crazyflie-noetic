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
    #                                               C O N S T R U C T O R
    #
    # This class completely handle one virtual swarm of crazyflies.
    # INPUTS:
    #   1) cf_names -> list of names of each Crazyflie within the virtual swarm;
    #
    # ==================================================================================================================
    def __init__(self, cf_names):
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                           P R O P E R T I E S  I N I T I A L I Z A T I O N
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        # Variable to understand when initial operations are ended (otherwise problem with 100Hz subscriber):
        self.__initialOperationsEnded = False

        # List of crazyflies names:
        self.cf_names = cf_names

        # Number of crazyflies:
        self.number_of_cfs = len(self.cf_names)

        # List of clients for takeoff action per each drone:
        self.takeoff_act_clients = []
        self.__make_takeoff_clients()

        # # List of clients for flocking action:
        # self.flocking_act_clients = []
        # self.__make_flocking_clients()

        # List of motion commanders initialization
        self.mc_list = []
        self.c_list = []

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       S U B S C R I B E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Subscriber to pace 100Hz:
        self.pace_100Hz_sub = rospy.Subscriber('/' + DEFAULT_100Hz_PACE_TOPIC, Empty, self.__pace_100Hz_sub_callback)

        # List of state subscribers:
        self.state_subs = []
        self.states = []
        self.__make_state_subs()
        
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       P U B L I S H E R S  S E T U P
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
        #                                           S E R V I C E S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                           A C T I O N S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Action to take off the entire swarm:
        self.__swarm_takeoff_act = actionlib.SimpleActionServer('/swarm/takeoff_actn', TakeoffAction,
                                                                self.__swarm_takeoff_act_callback, False)
        self.__swarm_takeoff_act.start()


        # Action to land the entire swarm:
        self.__swarm_land_act = actionlib.SimpleActionServer('/swarm/land_actn', TakeoffAction,
                                                                self.__swarm_land_act_callback, False)
        self.__swarm_land_act.start()



        # Flocking not implemented for real swarm yet
        # self.__swarm_flocking_act = actionlib.SimpleActionServer('/swarm/flocking_actn', EmptyAction,
        #                                                          self.__swarm_flocking_act_callback, False)
        # self.__swarm_flocking_act.start()


        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                        I N I T I A L  O P E R A T I O N S
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        # Drivers initialization:
        cflib.crtp.init_drivers()

        # List of URIs
        self.uris = self.compute_uris(cf_names)

        print(self.uris)

        # Instantiation of Swarm and opening communication:
        self.__swarm = Swarm(self.uris, factory = _Factory())
        self.__swarm.open_links()

        # Loggers configuration for all the agents

        self.__state_estimates = dict()
        self.__attitudes = dict()
        self.__controller_outputs = dict()
        self.__desired_states = dict()

        # for uri in self.uris:
        #     self.__swarm._cfs[uri] = self.__swarm.factory.construct(uri)

        # self.state_loggers_swarm()
        # self.attitude_loggers_swarm()
        # self.controller_output_loggers_swarm()
        # self.desired_state_loggers_swarm()

        self.create_commanders_swarm()

        self.__initialOperationsEnded = True
    





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
            print('added 1 takeoff client')

    # def __make_flocking_clients(self):
    #     for cf_name in self.cf_names:
    #         if cf_name != DEFAULT_LEADER:
    #             tmp_action = actionlib.SimpleActionClient('/' + cf_name + '/' + DEFAULT_FLOCK_TOPIC, EmptyAction)
    #             self.flocking_act_clients.append(tmp_action)
    #             self.flocking_act_clients[-1].wait_for_server()

    def __make_state_subs(self):
        for cf_name in self.cf_names:
            tmp_sub = rospy.Subscriber('/' + cf_name + '/' + 
                        DEFAULT_CF_STATE_TOPIC, CrazyflieState, self.__state_cb)
            self.state_subs.append(tmp_sub)
            self.states.append(CrazyflieState())

    def __make_controller_outputs_publishers(self):
        self.controller_outputs_pubs = []
        for cf_name in self.cf_names:
            tmp_pub = rospy.Publisher('/' + cf_name + '/' + 
                            DEFAULT_MOTOR_CMD_TOPIC, Attitude, queue_size=1)
            self.controller_outputs_pubs.append(tmp_pub)
            self.controller_outputs.append(Attitude())

    def __controller_outputs_pub(self, controller_outputs):
        index = 0
        for index, controller_output_pub in enumerate(self.controller_outputs_pubs):
            controller_output_pub.publish(controller_outputs.controller_outputs[index])
            # print('co pub is ok')

    def __make_desired_states_publishers(self):
        self.desired_states_pubs = []
        for cf_name in self.cf_names:
            tmp_pub = rospy.Publisher('/' + cf_name + '/' + 
                            DEFAULT_ACTUAL_DESTINATION_TOPIC, Position, queue_size=1)
            self.desired_states_pubs.append(tmp_pub)
            self.desired_states.append(Position())

    def __desired_states_pub(self, desired_states):
        index = 0
        for index, desired_state_pub in enumerate(self.desired_states_pubs):
            desired_state_pub.publish(desired_states.desired_states[index])
            # print('ds pub is ok')


    def compute_uris(self, cf_names):
        uris = []
        for name in cf_names:
            num_ID = int(name[2:]) - 1
            uri = 'radio://0/80/2M/E7E7E7E7E' + hex(num_ID)[-1]
            uris.append(uri)
        
        return uris

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

            # Configuring the loggers
            self.state_loggers_swarm()
            self.attitude_loggers_swarm()
            self.controller_output_loggers_swarm()
            self.desired_state_loggers_swarm()


            # Extracting information from the loggers

            cf_index = 0

            for uri in self.uris:
                # States
                self.states[cf_index].name = 'cf' + str(cf_index + 1)
                self.states[cf_index].position.x = self.__state_estimates[uri].x
                self.states[cf_index].position.y = self.__state_estimates[uri].y
                self.states[cf_index].position.z = self.__state_estimates[uri].z
                self.states[cf_index].velocity.x = self.__state_estimates[uri].vx
                self.states[cf_index].velocity.y = self.__state_estimates[uri].vy
                self.states[cf_index].velocity.z = self.__state_estimates[uri].vz

                # Attitudes
                self.states[cf_index].orientation.roll = self.__attitudes[uri].roll
                self.states[cf_index].orientation.pitch = self.__attitudes[uri].pitch
                self.states[cf_index].orientation.yaw = self.__attitudes[uri].yaw
                self.states[cf_index].rotating_speed.x = self.__attitudes[uri].gyro_x
                self.states[cf_index].rotating_speed.y = self.__attitudes[uri].gyro_y
                self.states[cf_index].rotating_speed.z = self.__attitudes[uri].gyro_z
                
                # Controller outputs
                self.controller_outputs[cf_index].desired_thrust = \
                                            self.__controller_outputs[uri].cmd_thrust
                self.controller_outputs[cf_index].desired_attitude.roll = \
                                            self.__controller_outputs[uri].cmd_roll
                self.controller_outputs[cf_index].desired_attitude.pitch = \
                                            self.__controller_outputs[uri].cmd_pitch
                self.controller_outputs[cf_index].desired_attitude.yaw = \
                                            self.__controller_outputs[uri].cmd_yaw

                # Desired states
                self.desired_states[cf_index].desired_velocity.x = \
                                    self.__desired_states[uri].target_vx
                self.desired_states[cf_index].desired_velocity.y = \
                                    self.__desired_states[uri].target_vy
                self.desired_states[cf_index].desired_velocity.z = \
                                    self.__desired_states[uri].target_vz
                self.desired_states[cf_index].desired_yaw_rate = \
                                    self.__desired_states[uri].yawrate
                
                cf_index += 1

            states.states = self.states
            controller_outputs.controller_outputs = self.controller_outputs
            desired_states.desired_states = self.desired_states

            # print(self.states)
            # print(self.controller_outputs)
            # print(self.desired_states)

            self.states_pub.publish(states)
            self.__controller_outputs_pub(controller_outputs)
            self.__desired_states_pub(desired_states)
            



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

        rospy.sleep(4)

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

        # # Setting up takeoff request:
        # _goal = TakeoffGoal()
        # _goal.takeoff_height = goal.takeoff_height

        # for takeoff_actn in self.takeoff_act_clients:
        #     takeoff_actn.send_goal(_goal, feedback_cb=self.__cf_takeoff_feedback_cb)
        #     #print('\n\nTIPO:' + str(type(takeoff_actn)))
        
        self.land_swarm()
        
        self.__swarm_land_act.set_succeeded(result)















    # def __swarm_flocking_act_callback(self, goal):
    #     # Defining the goal:
    #     flock_goal = EmptyGoal()

    #     name = []
    #     # Sending the goal to all followers clients:
    #     for ii in range(0, len(self.flocking_act_clients)):
    #         self.flocking_act_clients[ii].send_goal(flock_goal)

    #     # Sending result:
    #     result = EmptyResult()
    #     result.executed = True

    #     self.__swarm_flocking_act.set_succeeded(result)
    
    # ==================================================================================================================
    #
    #         F E E D B A C K  C A L L B A C K  M E T H O D S  (A C T I O N S)
    #
    # ==================================================================================================================
    def __cf_takeoff_feedback_cb(self, feedback):
        pass


    # ==================================================================================================================
    #
    #         L O G G E R S     C O N F I G U R A T I O N    
    #
    # ==================================================================================================================
  
    def state_logger_drone(self, scf):

        # State logger configuration (6 floats => 24/26 bytes):
        self.__state_logger_config = LogConfig(name='state_conf', period_in_ms=10)
        self.__state_logger_config.add_variable('stateEstimate.x', 'float')
        self.__state_logger_config.add_variable('stateEstimate.y', 'float')
        self.__state_logger_config.add_variable('stateEstimate.z', 'float')
        self.__state_logger_config.add_variable('stateEstimate.vx', 'float')
        self.__state_logger_config.add_variable('stateEstimate.vy', 'float')
        self.__state_logger_config.add_variable('stateEstimate.vz', 'float')

        with SyncLogger(scf, self.__state_logger_config) as logger:
            for entry in logger:
                x = entry[1]['stateEstimate.x']
                y = entry[1]['stateEstimate.y']
                z = entry[1]['stateEstimate.z']
                vx = entry[1]['stateEstimate.vx']
                vy = entry[1]['stateEstimate.vy']
                vz = entry[1]['stateEstimate.vz']
                self.__state_estimates[scf.cf.link_uri] = StateEstimateLog(x, y, z, 
                                                                        vx, vy, vz)
                break

    def state_loggers_swarm(self):
        """
        Configuring the loggers to be used 
        """
        self.__swarm.parallel_safe(self.state_logger_drone)
        return self.__state_estimates


  
    def attitude_logger_drone(self, scf):

        # Attitude logger configuration (6 floats => 24/26 bytes):
        self.__attitude_logger_config = LogConfig(name='attitude_conf', period_in_ms=10)
        self.__attitude_logger_config.add_variable('stabilizer.roll', 'float')
        self.__attitude_logger_config.add_variable('stabilizer.pitch', 'float')
        self.__attitude_logger_config.add_variable('stabilizer.yaw', 'float')
        self.__attitude_logger_config.add_variable('gyro.x', 'float')
        self.__attitude_logger_config.add_variable('gyro.y', 'float')
        self.__attitude_logger_config.add_variable('gyro.z', 'float')

        with SyncLogger(scf, self.__attitude_logger_config) as logger:
            for entry in logger:
                roll = deg2rad(entry[1]['stabilizer.roll'])
                pitch = deg2rad(-entry[1]['stabilizer.pitch'])
                yaw = deg2rad(entry[1]['stabilizer.yaw'])
                gyro_x = deg2rad(entry[1]['gyro.x'])
                gyro_y = deg2rad(entry[1]['gyro.y'])
                gyro_z = deg2rad(entry[1]['gyro.z'])
                self.__attitudes[scf.cf.link_uri] = AttitudeLog(roll, pitch,
                yaw, gyro_x, gyro_y, gyro_z)
                break

    def attitude_loggers_swarm(self):
        """
        Configuring the loggers to be used 
        """
        self.__swarm.parallel_safe(self.attitude_logger_drone)
        return self.__attitudes



  
    def controller_output_logger_drone(self, scf):

         # Controller output logger configuration (4 floats => 16/26 bytes):
        self.__controller_output_config = LogConfig(name='controller_output_conf', period_in_ms=10)
        self.__controller_output_config.add_variable('controller.cmd_thrust')
        self.__controller_output_config.add_variable('controller.cmd_roll')
        self.__controller_output_config.add_variable('controller.cmd_pitch')
        self.__controller_output_config.add_variable('controller.cmd_yaw')

        with SyncLogger(scf, self.__controller_output_config) as logger:
            for entry in logger:
                cmd_thrust = entry[1]['controller.cmd_thrust']
                cmd_roll = entry[1]['controller.cmd_roll']
                cmd_pitch = entry[1]['controller.cmd_pitch']
                cmd_yaw = entry[1]['controller.cmd_yaw']
                self.__controller_outputs[scf.cf.link_uri] = ControllerOutputLog(cmd_thrust, 
                cmd_roll, cmd_pitch, cmd_yaw)
                break

    def controller_output_loggers_swarm(self):
        """
        Configuring the loggers to be used 
        """
        self.__swarm.parallel_safe(self.controller_output_logger_drone)
        return self.__controller_outputs


  
    def desired_state_logger_drone(self, scf):

        # Reference velocity state logger configuration (4 floats => 16/26 bytes):
        self.__desired_state_logger_config = LogConfig(name='desired_state_config', period_in_ms=10)
        self.__desired_state_logger_config.add_variable('posCtl.targetVX', 'float')
        self.__desired_state_logger_config.add_variable('posCtl.targetVY', 'float')
        self.__desired_state_logger_config.add_variable('posCtl.targetVZ', 'float')
        self.__desired_state_logger_config.add_variable('controller.yawRate', 'float')

        with SyncLogger(scf, self.__desired_state_logger_config) as logger:
            for entry in logger:
                target_vx = entry[1]['posCtl.targetVX']
                target_vy = entry[1]['posCtl.targetVY']
                target_vz = entry[1]['posCtl.targetVZ']
                yawrate = entry[1]['controller.yawRate']
                self.__desired_states[scf.cf.link_uri] = RefVelocityLog(target_vx, target_vy,
                target_vz, yawrate)
                break

    def desired_state_loggers_swarm(self):
        """
        Configuring the loggers to be used 
        """
        self.__swarm.parallel_safe(self.desired_state_logger_drone)
        return self.__desired_states



    # ==================================================================================================================
    #
    #         M E T H O D S    F O R    A C T I O N S    
    #
    # ==================================================================================================================


    #+++++++++++ MOTION COMMANDER AND COMMANDER INSTANTIATION ++++++++++++++++++++

    # Creating a list of motion commanders to be used for the single drones
    # and for the swarm

    def create_commanders_drone(self,scf):
        print(scf)
        print(scf._link_uri)
        print(scf.cf)

        # Motion commander instance
        motion_commander = MotionCommander(scf)
        self.mc_list.append(motion_commander)

        print(motion_commander)
        print(self.mc_list)

        commander = Commander(scf.cf)
        self.c_list.append(commander)

        print(commander)
        print(self.c_list)

    def create_commanders_swarm(self):
        self.__swarm.parallel_safe(self.create_commanders_drone)

    #+++++++++++++++++++++++ TAKEOFF METHOD ++++++++++++++++++++++++++++++++++++++

    def takeoff_drone4swarm(self,scf):
        # uri = scf._link_uri
        # cf_index = int(uri[-1]) # ok for up to 10 drones
        # print('cf_index is:', cf_index)
        # self.mc_list[cf_index].take_off(height=0.3)


        print('instantiation of motion commander')
        motion_commander = MotionCommander(scf)
        print('land command')
        motion_commander.take_off(height=0.3)
        self.mc_list.append(motion_commander)

    def takeoff_swarm(self):
        print('takeoff action')

        self.mc_list = []
        self.__swarm.parallel_safe(self.takeoff_drone4swarm)


    # def takeoff_swarm(self):
    #     print('takeoff action')
    #     for cf_index in range(self.number_of_cfs):
    #         print('cf_index is: ', cf_index)
    #         self.mc_list[cf_index].take_off(height=0.3)

    #+++++++++++++++++++++++++ LAND METHOD +++++++++++++++++++++++++++++++++++++++

    # def land_drone4swarm(self,scf):
    #     print('instantiation of motion commander')
    #     motion_commander = MotionCommander(scf)
    #     print('land command')
    #     motion_commander.land()

    # def land_swarm(self):
    #     print('land action')        
    #     self.__swarm.parallel_safe(self.land_drone4swarm)



    def land_swarm(self):
        print('land action')
        for cf_index in range(self.number_of_cfs):
            print('cf_index is: ', cf_index)
            self.mc_list[cf_index].land()



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
        # # Land the drone:
        # self.__mc.land()

        # # Stop logger:
        # self.__attitude_logger.disconnect()

        print('about to close links')

        # Closing communication with the crazyflie:
        self.__swarm.close_links()

        print('links have been closed')

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                           __ E X I T __
    #
    # This method is called when an error occurs, performing some operations before destroy class instance.
    # ------------------------------------------------------------------------------------------------------------------
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.exit_operations()