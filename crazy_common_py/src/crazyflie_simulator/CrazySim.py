# ROS MODULES
import rospy

# GAZEBO MODULES
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest

class CrazySim:
    # ========================================================================================
    #
    #                                   C O N S T R U C T O R
    #
    # INPUTS:
    #   1) name -> name of the Crazyflie in the simulation
    # ========================================================================================
    def __init__(self, name):
        #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                           W A I T I N G  F O R  S E R V I C E S
        #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Waiting for service used to spawn the urdf model of Crazyflie in Gazebo:
        rospy.wait_for_service("/gazebo/spawn_urdf_model", SpawnModel)

        #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                   P R O P E R T I E S  I N I T I A L I Z A T I O N
        #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Name of the crazyflie in the simulation:
        self.name = name

        #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                           S U B S C R I B E R S  S E T U P
        #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                           P U B L I S H E R S  S E T U P
        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                               S E R V I C E S  S E T U P
        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Service used to spawn the urdf model of the Crazyflie in the simulation:
        self.spawn_model_service = rospy.ServiceProxy('/gazebo/spawn_urdf_model')
        self.__spawn_model_request_srv = SpawnModelRequest()

        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                               A C T I O N S  S E T U P
        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


        #------------------------------------------------------------------------------------------
        #                           I N I T I A L  O P E R A T I O N S
        #------------------------------------------------------------------------------------------
        self.__spawn_cf()

    def __spawn_cf(self):
        # Setting up the request message to spawn the crazyflie in the simulation:
        self.__spawn_model_request_srv.model_name = self.name
        self.__spawn_model_request_srv.model_xml = '~/catkin_ws/src/crazyflie_description/urdf/crazyflie_basic.urdf'
        self.__spawn_model_request_srv.robot_namespace = self.name
        self.__spawn_model_request_srv.initial_pose.position.x = 0.0
        self.__spawn_model_request_srv.initial_pose.position.y = 0.0
        self.__spawn_model_request_srv.initial_pose.position.z = 0.2
        self.__spawn_model_request_srv.initial_pose.orientation.x = 0.0
        self.__spawn_model_request_srv.initial_pose.orientation.y = 0.0
        self.__spawn_model_request_srv.initial_pose.orientation.z = 0.0
        self.__spawn_model_request_srv.initial_pose.orientation.w = 0.0

        # Calling the service to spawn the model:
        self.spawn_model_service(self.__spawn_model_request_srv)
        pass
