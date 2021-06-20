# ROS MODULES
import rospy
import roslaunch
# GAZEBO MODULES
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest

# CUSTOM MODULES
from crazy_common_py.dataTypes import Vector3

class CrazySim:
    # ==================================================================================================================
    #
    #                                               C O N S T R U C T O R
    #
    # INPUTS:
    #   1) name -> name of the Crazyflie in the simulation
    # ==================================================================================================================
    def __init__(self, name, initialPosition):
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
        self.__spawn_cf()
        self.__set_control_parameters()
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
        # Setting up the request message to spawn the crazyflie in the simulation:
        self.__spawn_model_request_srv.model_name = self.name
        self.__spawn_model_request_srv.model_xml = self.__setup_custom_urdf(
            '/home/andrea/catkin_ws/src/crazyflie_description/urdf/crazyflie_basic.urdf')
        self.__spawn_model_request_srv.robot_namespace = self.name
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
        rospy.set_param('/' + self.name + '/joint_state_controller/publish_rate', 50)

        # Setting up rosparameters used by velocity controllers of motor joints in Gazebo:
        self.__set_motor_parameters('crazyflie_M1_joint_velocity_controller', 'crazyflie_M1_joint', 1.0, 1.0, 0.0,
                                    100.0)
        self.__set_motor_parameters('crazyflie_M2_joint_velocity_controller', 'crazyflie_M2_joint', 1.0, 1.0, 0.0,
                                    100.0)
        self.__set_motor_parameters('crazyflie_M3_joint_velocity_controller', 'crazyflie_M3_joint', 1.0, 1.0, 0.0,
                                    100.0)
        self.__set_motor_parameters('crazyflie_M4_joint_velocity_controller', 'crazyflie_M4_joint', 1.0, 1.0, 0.0,
                                    100.0)

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
    # This method is used to modify the urdf of the model to spawn, since in the "PLUGINS" section we need a unique
    # <robotNamespace> tag, which is set equal to the name of the spawned crazyflie.
    # ------------------------------------------------------------------------------------------------------------------
    def __setup_custom_urdf(self, urdf_path):
        # Saving all the file as a string:
        urdf_file = open(urdf_path).read()

        # Finding and modifying robot namespace:
        initial_pos = urdf_file.find('/cf1')
        custom_urdf_file = urdf_file[:initial_pos]
        custom_urdf_file = custom_urdf_file + '/' + self.name + urdf_file[initial_pos+4:]
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

