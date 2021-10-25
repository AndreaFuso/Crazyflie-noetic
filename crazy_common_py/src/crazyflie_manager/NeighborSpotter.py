# ROS MODULES
import rospy
import actionlib
import math
# CUSTOM MODULES
from crazy_common_py.default_rosparameters import DEFAULT_ROSPARAM_NUMBER_OF_CFS
from crazy_common_py.constants import DEFAULT_NAME, DEFAULT_RADIUS_SS, MAX_VELOCITY_X, MAX_VELOCITY_Y, MAX_VELOCITY_Z, \
    DEFAULT_SAFETY_RADIUS_SS
from crazy_common_py.default_topics import DEFAULT_CF_STATE_TOPIC, DEFAULT_100Hz_PACE_TOPIC

from crazyflie_messages.msg import CrazyflieState
from crazyflie_messages.msg import GetStateAction, GetStateResult, GetStateGoal

from std_msgs.msg import Empty

from crazy_common_py.common_functions import extractCfNumber, standardNameList, constrain
from crazy_common_py.dataTypes import Vector3, SphericalSpotter, SpotterType

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
    def __init__(self, cfName, spotter_type=SphericalSpotter(DEFAULT_RADIUS_SS)):
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                           P R O P E R T I E S  I N I T I A L I Z A T I O N
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Name of the crazyflie:
        self.name = cfName

        # List containing all neighbors:
        self.__actual_neighbors = []

        # List of state subscribers:
        self.__state_subscribers = []

        # Getting number of cfs within the swarm:
        self.__number_of_cfs = rospy.get_param(DEFAULT_ROSPARAM_NUMBER_OF_CFS)

        # List of all states:
        self.__states = []

        # Position in list state of reference crazyflie:
        self.__this_state_pos = int(extractCfNumber(cfName)) - 1

        # Spotter:
        self.__spotter = spotter_type

        # Desired velocity:
        self.desired_velocity = Vector3()

        # Variable to understand if the spotter is activated (to avoid useless calculations):
        self.isActivated = False

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                    S U B S C R I B E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
        # Actions to get all swarm states:
        self.__make_state_subs()

        # Subscriber to pace 100Hz:
        self.pace_100Hz_sub = rospy.Subscriber('/' + DEFAULT_100Hz_PACE_TOPIC, Empty, self.__pace_100Hz_sub_callback)

    # ==================================================================================================================
    #
    #                                I N I T I A L  O P E R A T I O N S  M E T H O D S
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                  __M A K E _ S T A T E _ S U B S
    #
    # This method is used to initialize the list containing all state subscribers (one for each drone in the swarm).
    # ------------------------------------------------------------------------------------------------------------------
    def __make_state_subs(self):
        cont = 1
        for ii in range(0, self.__number_of_cfs):
            tmp_sub = rospy.Subscriber('/' + DEFAULT_NAME + str(cont) + '/' + DEFAULT_CF_STATE_TOPIC, CrazyflieState,
                                       self.__state_sub_callback)
            self.__state_subscribers.append(tmp_sub)
            # Initialize also
            self.__states.append(CrazyflieState())
            cont += 1

    # ==================================================================================================================
    #
    #                                     C A L L B A C K  M E T H O D S  (T O P I C S)
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                  __S T A T E _ S U B _ C A L L B A C K
    #
    # This callback is called whenever a state is published.
    # ------------------------------------------------------------------------------------------------------------------
    def __state_sub_callback(self, msg):
        # Getting crazyflie number:
        cf_number = extractCfNumber(msg.name)

        # Updating its state:
        self.__states[cf_number - 1] = msg

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                  __P A C E _ 1 0 0 H Z _ S U B _ C A L L B A C K
    #
    # This callback is called whenever an Empty message is published by pace_100Hz_node; it is used to develop the main
    # routine.
    # ------------------------------------------------------------------------------------------------------------------
    def __pace_100Hz_sub_callback(self, msg):
        if self.isActivated:
            # Getting actual states:
            states = self.__states[:]

            # Identify crazyflie reference state:
            cf_ref_state = states[self.__this_state_pos]

            # Updating neighbors:
            self.__update_neighbors(states, cf_ref_state)

            # Calculating velocity components:
            #                                           v_des = v_a + v_c + v_s
            #       - v_a = alignment component;
            #       - v_c = cohesion component;
            #       - v_s = separation component;
            self.desired_velocity = self.__compute_desired_velocity(states, cf_ref_state, DEFAULT_SAFETY_RADIUS_SS)

        #TODO: (1) calcolare grandezze relative; ,
        #TODO: (2) pubblicare informazioni (utile creare nuovo messaggio tipo Neighbor.msg, che comprende tutte le informazioni del vicino e pubblicare il tutto con un tipo ArrayNeighbors.msg)
        #TODO:      serve publicarle? Alla fine non conviene buttarle fuori con un metodo (fuori potrei mettere la classe BoidLocalManager) y

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

    # ==================================================================================================================
    #
    #                                           G E N E R A L  M E T H O D S
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                           __U P D A T E _ N E I G H B O R S
    #
    # This method is use to update the "actual_neighbors" list, to understand who is within defined horizon.
    # ------------------------------------------------------------------------------------------------------------------
    def __update_neighbors(self, states, cf_ref_state):
        # Cleaning the neighbors list:
        self.__actual_neighbors = []

        # Cycle to understand the neighbors:
        for state in states:
            if state.name != self.name:
                # SPHERICAL SPOTTER
                if self.__spotter.getType() == SpotterType.SPHERICAL:
                    if self.__spotter.isContained(Vector3(cf_ref_state.position.x, cf_ref_state.position.y,
                                                          cf_ref_state.position.z),
                                                  Vector3(state.position.x, state.position.y, state.position.z)):
                        self.__actual_neighbors.append(state)
                elif self.__spotter.getType() == SpotterType.BOX:
                    rospy.logerr('BOX SPOTTER TYPE NOT DEFINED')
                else:
                    rospy.logerr('UNDEFINED SPOTTER TYPE')
        return

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                   __C O M P U T E _ D E S I R E D _ V E L O C I T Y
    #
    # This method is use to comp[ute desired velocity.
    # ------------------------------------------------------------------------------------------------------------------
    def __compute_desired_velocity(self, states, cf_ref_state, safety_radius=DEFAULT_SAFETY_RADIUS_SS,
                                   w_a=1.6, w_c=1.6, w_s=0.6):
        # Number of crazyflies within horizion:
        number_of_cfs = len(self.__actual_neighbors)

        # If there's no crazyflie within horizon stop:
        if number_of_cfs == 0:
            #TODO: CALL STOP ACTION FOR ACTUAL CRAZYFLIE + PREEMP FOLLOWER ACTION?
            return Vector3(0.0, 0.0, 0.0)

        else:
            # CYCLE TO OBTAIN COMMON QUANTITIES
            # v_a components initialization:
            va_x = 0
            va_y = 0
            va_z = 0

            # x_m components initialization:
            xm_x = 0
            xm_y = 0
            xm_z = 0

            # x_diff components initialization (xi - xj):
            x_diff_x = 0
            x_diff_y = 0
            x_diff_z = 0

            for ii in range(0, number_of_cfs):
                # Updating v_a components:
                va_x += self.__actual_neighbors[ii].velocity.x
                va_y += self.__actual_neighbors[ii].velocity.y
                va_z += self.__actual_neighbors[ii].velocity.z

                # Updating x_m to save some computational effort:
                xm_x += self.__actual_neighbors[ii].position.x
                xm_y += self.__actual_neighbors[ii].position.y
                xm_z += self.__actual_neighbors[ii].position.z

                # Calculating vector difference (xi - xj, being i this crazyflie):
                x_diff_x_tmp = cf_ref_state.position.x - self.__actual_neighbors[ii].position.x
                x_diff_y_tmp = cf_ref_state.position.y - self.__actual_neighbors[ii].position.y
                x_diff_z_tmp = cf_ref_state.position.z - self.__actual_neighbors[ii].position.z

                # Calculating norm:
                x_diff_norm = math.sqrt(x_diff_x_tmp ** 2 + x_diff_y_tmp ** 2 + x_diff_z_tmp ** 2)

                # Updating diff components:
                x_diff_x += (1 / (x_diff_norm - safety_radius)) * (x_diff_x_tmp / x_diff_norm)
                x_diff_y += (1 / (x_diff_norm - safety_radius)) * (x_diff_y_tmp / x_diff_norm)
                x_diff_z += (1 / (x_diff_norm - safety_radius)) * (x_diff_z_tmp / x_diff_norm)

            # ALIGNMENT COMPONENT
            # Calculating velocity alignment components:
            va_x_mean = va_x / number_of_cfs
            va_y_mean = va_y / number_of_cfs
            va_z_mean = va_z / number_of_cfs

            va_mean_norm = math.sqrt(va_x_mean ** 2 + va_y_mean ** 2 + va_z_mean ** 2)

            va_x = w_a * va_mean_norm * va_x_mean
            va_y = w_a * va_mean_norm * va_y_mean
            va_z = w_a * va_mean_norm * va_z_mean

            # Getting x_m components:
            xm_x = xm_x / number_of_cfs
            xm_y = xm_y / number_of_cfs
            xm_z = xm_z / number_of_cfs

            # COHESION COMPONENT
            # Calculating the norm of difference xm - xi (being i this crazyflie):
            xm_xi_norm = math.sqrt((xm_x - cf_ref_state.position.x) ** 2 + (xm_y - cf_ref_state.position.y) ** 2 +
                                   (xm_z - cf_ref_state.position.z) ** 2)

            # Calculating velocity cohesion components:
            vc_x = w_c * xm_xi_norm * (xm_x - cf_ref_state.position.x)
            vc_y = w_c * xm_xi_norm * (xm_y - cf_ref_state.position.y)
            vc_z = w_c * xm_xi_norm * (xm_z - cf_ref_state.position.z)

            # SEPARATION COMPONENT
            vs_x = w_s * x_diff_x
            vs_y = w_s * x_diff_y
            vs_z = w_s * x_diff_z

            # DESIRED VELOCITY
            v_des_x = constrain(va_x + vc_x + vs_x, - MAX_VELOCITY_X, MAX_VELOCITY_X)
            v_des_y = constrain(va_y + vc_y + vs_y, - MAX_VELOCITY_Y, MAX_VELOCITY_Y)
            v_des_z = constrain(va_z + vc_z + vs_z, - MAX_VELOCITY_Z, MAX_VELOCITY_Z)

            return Vector3(v_des_x, v_des_y, v_des_z)