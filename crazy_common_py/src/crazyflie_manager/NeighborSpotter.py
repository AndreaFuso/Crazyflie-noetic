# ROS MODULES
import rospy
import actionlib
# CUSTOM MODULES
from crazy_common_py.default_rosparameters import DEFAULT_ROSPARAM_NUMBER_OF_CFS
from crazy_common_py.constants import DEFAULT_NAME
from crazy_common_py.default_topics import DEFAULT_CF_STATE_TOPIC, DEFAULT_100Hz_PACE_TOPIC

from crazyflie_messages.msg import CrazyflieState
from std_msgs.msg import Empty

from crazy_common_py.common_functions import extractCfNumber
from crazy_common_py.dataTypes import Vector3, SphericalSpotter

class NeighborSpotter:
    # ==================================================================================================================
    #
    #                                               C O N S T R U C T O R
    #
    # This class is used to understand which other crazyflies are close and how they are moving with respect the
    # considered one.
    # INPUTS:
    #   1) cfName -> name of the crazyflie in the simulation;
    # ==================================================================================================================
    def __init__(self, cfName, spotter_type=SphericalSpotter()):
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                           P R O P E R T I E S  I N I T I A L I Z A T I O N
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Name of the crazyflie:
        self.name = cfName

        # List containing all neighbors:
        self.actual_neighbors = []

        # List of state subscribers:
        self.__state_subscribers = []

        # Getting number of cfs within the swarm:
        self.__number_of_cfs = rospy.get_param(DEFAULT_ROSPARAM_NUMBER_OF_CFS)

        # List of all states:
        self.__states = []

        # Position in list state of reference crazyflie:
        self.__this_state_pos = extractCfNumber(cfName) - 1

        # Spotter:
        self.spotter = spotter_type

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                    S U B S C R I B E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Subscribers to get all swarm states:
        for ii in range(0, self.__number_of_cfs):
            tmp_sub = rospy.Subscriber('/' + DEFAULT_NAME + '/' + DEFAULT_CF_STATE_TOPIC, CrazyflieState,
                                       self.__state_sub_callback)
            self.__state_subscribers.append(tmp_sub)
            # Initialize also
            self.__states.append(CrazyflieState())

        # Subscriber to pace 100Hz:
        self.pace_100Hz_sub = rospy.Subscriber('/' + DEFAULT_100Hz_PACE_TOPIC, Empty, self.__pace_100Hz_sub_callback)

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       P U B L I S H E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       S E R V I C E S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                        A C T I O N S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    # ==================================================================================================================
    #
    #                                     C A L L B A C K  M E T H O D S  (T O P I C S)
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                  __S T A T E _ S U B _ C A L L B A C K
    #
    # This callback is called whenever a state is published:
    # ------------------------------------------------------------------------------------------------------------------
    def __state_sub_callback(self, msg):
        # Getting crazyflie number:
        cf_number = extractCfNumber(msg.name)

        # Updating its state:
        self.__states[cf_number - 1] = msg

    def __pace_100Hz_sub_callback(self, msg):
        # Saving all actual states:
        actual_states = self.__states[:]

        # Identify crazyflie reference state:
        cf_ref_state = actual_states[self.__this_state_pos]

        # Cleaning the list:
        self.actual_neighbors = []

        # Cycle to understand the neighbors:
        for state in actual_states:
            if state.name != self.name:
                if self.spotter.isContained(Vector3(cf_ref_state.position.x, cf_ref_state.position.y,
                                                    cf_ref_state.position.z),
                                            Vector3(state.position.x, state.position.y, state.position.z)):
                    self.actual_neighbors.append(state)

        #TODO: (1) calcolare grandezze relative; ,
        #TODO: (2) pubblicare informazioni (utile creare nuovo messaggio tipo Neighbor.msg, che comprende tutte le informazioni del vicino e pubblicare il tutto con un tipo ArrayNeighbors.msg)
        #TODO:      serve publicarle? Alla fine non conviene buttarle fuori con un metodo (fuori potrei mettere la classe BoidLocalManager)

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

    # ==================================================================================================================
    #
    #                            F E E D B A C K  C A L L B A C K  M E T H O D S  (A C T I O N S)
    #
    # ==================================================================================================================