# ROS MODULES
import rospy
import roslaunch
import rospkg

# GAZEBO MODULES
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest

import sys
import xacro
import subprocess
import time

# CUSTOM MODULES
from crazy_common_py.dataTypes import Vector3, GazeboIMU
from crazy_common_py.constants import *
from crazyflie_simulator.pid import *
from crazyflie_simulator.FlightControllerSimFirmwr import FlightControllerSimFirmwr
from crazyflie_simulator.MyFlightControllerFirmwr import MyFlightControllerFirmwr
from crazyflie_simulator.FlightControllerSimCustom import FlightControllerCustom
from crazyflie_simulator.StateEstimatorSim import FakeStateEstimator
from crazyflie_simulator.MotionCommanderSim import MotionCommanderSim
from crazy_common_py.default_topics import DEFAULT_ODOMETRY_TOPIC, DEFAULT_FORCE_STATE_TOPIC_M1, \
    DEFAULT_FORCE_STATE_TOPIC_M2, DEFAULT_FORCE_STATE_TOPIC_M3, DEFAULT_FORCE_STATE_TOPIC_M4

# OTHER MODULES
import os

class CrazySim:
    # ==================================================================================================================
    #
    #                                               C O N S T R U C T O R
    #
    # This class completely handle one virtual Crazyflie.
    # INPUTS:
    #   1) name -> name of the Crazyflie in the simulation;
    #   2) initialPosition -> Vector3 object with the inital position coordinates;
    # ==================================================================================================================
    def __init__(self, name, initialPosition, testBench=False):
        #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                               W A I T I N G  F O R  S E R V I C E S
        #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Waiting for service used to spawn the urdf model of Crazyflie in Gazebo:
        rospy.wait_for_service("/gazebo/spawn_urdf_model")

        #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                           P R O P E R T I E S  I N I T I A L I Z A T I O N
        #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Name of the crazyflie in the simulation:
        self.name = name

        # Initial position of the crazyflie
        self.__initial_position = initialPosition

        # Simulated IMU parameters:
        self.IMU = GazeboIMU(IMU_GAUSSIAN_NOISE_DEFAULT, IMU_UPDATE_RATE_DEFAULT)
        self.state_estimator = FakeStateEstimator(name)

        # Istance of motion commander:
        self.motion_commander = MotionCommanderSim(name)

        # Test bench choice:
        self.testBench = testBench

        #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       S U B S C R I B E R S  S E T U P
        #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       P U B L I S H E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                           S E R V I C E S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Service used to spawn the urdf model of the Crazyflie in the simulation:
        self.spawn_model_service = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        self.__spawn_model_request_srv = SpawnModelRequest()

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                           A C T I O N S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


        #---------------------------------------------------------------------------------------------------------------
        #                                       I N I T I A L  O P E R A T I O N S
        #---------------------------------------------------------------------------------------------------------------
        self.__set_robot_description_parameter()
        self.__set_control_parameters()
        self.__spawn_cf()
        self.__start_gazebo_controllers()

    # ==================================================================================================================
    #
    #                                   I N I T I A L  O P E R A T I O N S  M E T H O D S
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                           __S P A W N  C F
    #
    # This method is used to spawn a virtual crazyflie in the actual opened Gazebo world.
    # ------------------------------------------------------------------------------------------------------------------
    def __spawn_cf(self):
        # Getting the urdf path:
        rospack = rospkg.RosPack()
        if not self.testBench:
            urdf_path = rospack.get_path('crazyflie_description') + '/urdf/crazyflie_reference.urdf'
        else:
            urdf_path = rospack.get_path('crazyflie_description') + '/urdf/crazyflie_test_bench.urdf'

        # Setting up the request message to spawn the crazyflie in the simulation:
        self.__spawn_model_request_srv.model_name = self.name
        self.__spawn_model_request_srv.model_xml = self.__setup_custom_urdf(urdf_path)
        self.__spawn_model_request_srv.robot_namespace = self.name

        if not self.testBench:
            self.__spawn_model_request_srv.initial_pose.position.x = self.__initial_position.x
            self.__spawn_model_request_srv.initial_pose.position.y = self.__initial_position.y
            self.__spawn_model_request_srv.initial_pose.position.z = self.__initial_position.z
            self.__spawn_model_request_srv.initial_pose.orientation.x = 0.0
            self.__spawn_model_request_srv.initial_pose.orientation.y = 0.0
            self.__spawn_model_request_srv.initial_pose.orientation.z = 0.0
            self.__spawn_model_request_srv.initial_pose.orientation.w = 0.0

        # Calling the service to spawn the model:
        self.spawn_model_service(self.__spawn_model_request_srv)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                               __S E T  C O N T R O L  P A R A M E T E R S
    #
    # This method is used to create the parameters required for the creation of Gazebo velocity controllers of the
    # simulated rotors.
    # ------------------------------------------------------------------------------------------------------------------
    def __set_control_parameters(self):
        # Joint State Controller parameters:
        rospy.set_param('/' + self.name + '/joint_state_controller/type', 'joint_state_controller/JointStateController')
        rospy.set_param('/' + self.name + '/joint_state_controller/publish_rate', 500)

        # Setting up rosparameters used by velocity controllers of motor joints in Gazebo:
        self.__set_motor_parameters('crazyflie_M1_joint_velocity_controller', 'crazyflie_M1_joint',
                                    ACTUATOR_VELOCITY_CONTROLLER_KP, ACTUATOR_VELOCITY_CONTROLLER_KI,
                                    ACTUATOR_VELOCITY_CONTROLLER_KD, ACTUATOR_VELOCITY_CONTROLLER_I_CLAMP)
        self.__set_motor_parameters('crazyflie_M2_joint_velocity_controller', 'crazyflie_M2_joint',
                                    ACTUATOR_VELOCITY_CONTROLLER_KP, ACTUATOR_VELOCITY_CONTROLLER_KI,
                                    ACTUATOR_VELOCITY_CONTROLLER_KD, ACTUATOR_VELOCITY_CONTROLLER_I_CLAMP)
        self.__set_motor_parameters('crazyflie_M3_joint_velocity_controller', 'crazyflie_M3_joint',
                                    ACTUATOR_VELOCITY_CONTROLLER_KP, ACTUATOR_VELOCITY_CONTROLLER_KI,
                                    ACTUATOR_VELOCITY_CONTROLLER_KD, ACTUATOR_VELOCITY_CONTROLLER_I_CLAMP)
        self.__set_motor_parameters('crazyflie_M4_joint_velocity_controller', 'crazyflie_M4_joint',
                                    ACTUATOR_VELOCITY_CONTROLLER_KP, ACTUATOR_VELOCITY_CONTROLLER_KI,
                                    ACTUATOR_VELOCITY_CONTROLLER_KD, ACTUATOR_VELOCITY_CONTROLLER_I_CLAMP)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                          __S E T  R O B O T  D E S C R I P T I O N  P A R A M E T E R
    #
    # This method is used to create the parameter containing the urdf info used by gazebo ros control plugin.
    # ------------------------------------------------------------------------------------------------------------------
    def __set_robot_description_parameter(self):
        # Getting the urdf path:
        rospack = rospkg.RosPack()
        if not self.testBench:
            urdf_path = rospack.get_path('crazyflie_description') + '/urdf/crazyflie_reference.urdf'
        else:
            urdf_path = rospack.get_path('crazyflie_description') + '/urdf/crazyflie_test_bench.urdf'
        # Setting up the ros parameter:
        robot_description = self.__setup_custom_urdf(urdf_path)
        rospy.set_param('/robot_description', robot_description)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                               __S T A R T  G A Z E B O  C O N T R O L L E R S
    #
    # This method is used to launch the node that generates the topic through which is possible to set the reference
    # rotating speed for each rotor of the simulated crazyflie.
    # ------------------------------------------------------------------------------------------------------------------
    def __start_gazebo_controllers(self):
        node = roslaunch.core.Node(package='controller_manager', name='controller_spawner', node_type='spawner',
                                   namespace='/'+self.name,
                                   args='crazyflie_M1_joint_velocity_controller crazyflie_M2_joint_velocity_controller crazyflie_M3_joint_velocity_controller crazyflie_M4_joint_velocity_controller joint_state_controller')
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        process = launch.launch(node)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                   __S E T U P  C U S T O M  U R D F
    #
    # This method is used to modify the urdf of the model to spawn, in particular:
    #   1) in the "PLUGINS" section we need a unique <robotNamespace> tag, which is set equal to the name of the
    #   spawned crazyflie;
    #   2) change the path where the .dae parts are placed;
    # ------------------------------------------------------------------------------------------------------------------
    def __setup_custom_urdf(self, urdf_path):
        # Saving all the file as a string:
        urdf_file = open(urdf_path).read()

        # Setting up default paths for meshes:
        rospack = rospkg.RosPack()
        basic_link_mesh_path = rospack.get_path('crazyflie_description') + '/meshes/crazyflie2.dae'
        cw_propeller_mesh_path = rospack.get_path('crazyflie_description') + '/meshes/propeller_cw.dae'
        ccw_propeller_mesh_path = rospack.get_path('crazyflie_description') + '/meshes/propeller_ccw.dae'

        # Finding and modifying the path of the base link .dae and robotNamespace tag:
        tag_bl = 'PATH_BASE'
        tag_p1 = 'PATH_PROP_1'
        tag_p2 = 'PATH_PROP_2'
        tag_p3 = 'PATH_PROP_3'
        tag_p4 = 'PATH_PROP_4'
        tag_ns = 'ROBOT_NAMESPACE'
        tag_odom = 'ODOMETRY_TOPIC'
        tag_imu_gn = 'IMU_GAUSSIAN_NOISE_VALUE'
        tag_imu_ur = 'IMU_UPDATE_RATE'

        tag_freq_int_1 = 'FORCE_STATE_FREQ_INT_1'
        tag_freq_float_1 = 'FORCE_STATE_FREQ_FLOAT_1'
        tag_fs_m1 = 'FORCE_STATE_TOPIC_M1'

        tag_freq_int_2 = 'FORCE_STATE_FREQ_INT_2'
        tag_freq_float_2 = 'FORCE_STATE_FREQ_FLOAT_2'
        tag_fs_m2 = 'FORCE_STATE_TOPIC_M2'

        tag_freq_int_3 = 'FORCE_STATE_FREQ_INT_3'
        tag_freq_float_3 = 'FORCE_STATE_FREQ_FLOAT_3'
        tag_fs_m3 = 'FORCE_STATE_TOPIC_M3'

        tag_freq_int_4 = 'FORCE_STATE_FREQ_INT_4'
        tag_freq_float_4 = 'FORCE_STATE_FREQ_FLOAT_4'
        tag_fs_m4 = 'FORCE_STATE_TOPIC_M4'

        initial_pos_bl = urdf_file.find(tag_bl)
        initial_pos_p1 = urdf_file.find(tag_p1)
        initial_pos_p2 = urdf_file.find(tag_p2)
        initial_pos_p3 = urdf_file.find(tag_p3)
        initial_pos_p4 = urdf_file.find(tag_p4)
        initial_pos_ns = urdf_file.find(tag_ns)
        initial_pos_odom = urdf_file.find(tag_odom)
        initial_pos_imu_gn = urdf_file.find(tag_imu_gn)
        initial_pos_imu_ur = urdf_file.find(tag_imu_ur)

        initial_pos_freq_int_1 = urdf_file.find(tag_freq_int_1)
        initial_pos_freq_float_1 = urdf_file.find(tag_freq_float_1)
        initial_pos_fs_m1 = urdf_file.find(tag_fs_m1)

        initial_pos_freq_int_2 = urdf_file.find(tag_freq_int_2)
        initial_pos_freq_float_2 = urdf_file.find(tag_freq_float_2)
        initial_pos_fs_m2 = urdf_file.find(tag_fs_m2)

        initial_pos_freq_int_3 = urdf_file.find(tag_freq_int_3)
        initial_pos_freq_float_3 = urdf_file.find(tag_freq_float_3)
        initial_pos_fs_m3 = urdf_file.find(tag_fs_m3)

        initial_pos_freq_int_4 = urdf_file.find(tag_freq_int_4)
        initial_pos_freq_float_4 = urdf_file.find(tag_freq_float_4)
        initial_pos_fs_m4 = urdf_file.find(tag_fs_m4)

        custom_urdf_file = urdf_file[:initial_pos_bl]
        custom_urdf_file = custom_urdf_file + basic_link_mesh_path + urdf_file[initial_pos_bl + len(tag_bl):initial_pos_p1]
        custom_urdf_file = custom_urdf_file + ccw_propeller_mesh_path + urdf_file[initial_pos_p1 + len(tag_p1):initial_pos_p2]
        custom_urdf_file = custom_urdf_file + cw_propeller_mesh_path + urdf_file[initial_pos_p2 + len(tag_p2):initial_pos_p3]
        custom_urdf_file = custom_urdf_file + ccw_propeller_mesh_path + urdf_file[initial_pos_p3 + len(tag_p3):initial_pos_p4]
        custom_urdf_file = custom_urdf_file + ccw_propeller_mesh_path + urdf_file[initial_pos_p4 + len(tag_p4):initial_pos_ns]

        custom_urdf_file = custom_urdf_file + self.name + urdf_file[initial_pos_ns + len(tag_ns):initial_pos_odom]

        custom_urdf_file = custom_urdf_file + DEFAULT_ODOMETRY_TOPIC + urdf_file[initial_pos_odom + len(tag_odom):initial_pos_imu_gn]

        custom_urdf_file = custom_urdf_file + str(self.IMU.gaussian_noise) + urdf_file[initial_pos_imu_gn + len(tag_imu_gn):initial_pos_imu_ur]
        custom_urdf_file = custom_urdf_file + str(self.IMU.update_rate) + urdf_file[initial_pos_imu_ur + len(tag_imu_ur):initial_pos_freq_int_1]

        custom_urdf_file = custom_urdf_file + str(int(FORCE_STATE_FREQ_UPDATE)) + urdf_file[initial_pos_freq_int_1 + len(tag_freq_int_1):initial_pos_freq_float_1]
        custom_urdf_file = custom_urdf_file + str(float(FORCE_STATE_FREQ_UPDATE)) + urdf_file[initial_pos_freq_float_1 + len(tag_freq_float_1):initial_pos_fs_m1]
        custom_urdf_file = custom_urdf_file + DEFAULT_FORCE_STATE_TOPIC_M1 + urdf_file[initial_pos_fs_m1 + len(tag_fs_m1):initial_pos_freq_int_2]

        custom_urdf_file = custom_urdf_file + str(int(FORCE_STATE_FREQ_UPDATE)) + urdf_file[initial_pos_freq_int_2 + len(tag_freq_int_2):initial_pos_freq_float_2]
        custom_urdf_file = custom_urdf_file + str(float(FORCE_STATE_FREQ_UPDATE)) + urdf_file[initial_pos_freq_float_2 + len(tag_freq_float_2):initial_pos_fs_m2]
        custom_urdf_file = custom_urdf_file + DEFAULT_FORCE_STATE_TOPIC_M2 + urdf_file[initial_pos_fs_m2 + len(tag_fs_m2):initial_pos_freq_int_3]

        custom_urdf_file = custom_urdf_file + str(int(FORCE_STATE_FREQ_UPDATE)) + urdf_file[initial_pos_freq_int_3 + len(tag_freq_int_3):initial_pos_freq_float_3]
        custom_urdf_file = custom_urdf_file + str(float(FORCE_STATE_FREQ_UPDATE)) + urdf_file[initial_pos_freq_float_3 + len(tag_freq_float_3):initial_pos_fs_m3]
        custom_urdf_file = custom_urdf_file + DEFAULT_FORCE_STATE_TOPIC_M3 + urdf_file[initial_pos_fs_m3 + len(tag_fs_m3):initial_pos_freq_int_4]

        custom_urdf_file = custom_urdf_file + str(int(FORCE_STATE_FREQ_UPDATE)) + urdf_file[initial_pos_freq_int_4 + len(tag_freq_int_4):initial_pos_freq_float_4]
        custom_urdf_file = custom_urdf_file + str(float(FORCE_STATE_FREQ_UPDATE)) + urdf_file[initial_pos_freq_float_4 + len(tag_freq_float_4):initial_pos_fs_m4]
        custom_urdf_file = custom_urdf_file + DEFAULT_FORCE_STATE_TOPIC_M4 + urdf_file[initial_pos_fs_m4 + len(tag_fs_m4):]

        return custom_urdf_file

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                   __S E T  M O T O R  P A R A M E T E R S
    #
    # This method is used to set up all the parameters required by the Gazebo velocity controller for a single motor.
    # ------------------------------------------------------------------------------------------------------------------
    def __set_motor_parameters(self, controllerName, jointName, p, i, d, i_clamp):
        rospy.set_param('/' + self.name + '/' + controllerName + '/type',
                        'velocity_controllers/JointVelocityController')
        rospy.set_param('/' + self.name + '/' + controllerName + '/joint', jointName)
        rospy.set_param('/' + self.name + '/' + controllerName + '/pid/p', p)
        rospy.set_param('/' + self.name + '/' + controllerName + '/pid/i', i)
        rospy.set_param('/' + self.name + '/' + controllerName + '/pid/d', d)
        rospy.set_param('/' + self.name + '/' + controllerName + '/pid/i_clamp', i_clamp)

