# ROS MODULES
import rospy
import actionlib
# CUSTOM MODULES

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
    def __init__(self, cfName, spotter_type):
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                           P R O P E R T I E S  I N I T I A L I Z A T I O N
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Name of the crazyflie:
        self.name = cfName

        # List containing all neighbors:
        self.actual_neighbors = []

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                    S U B S C R I B E R S  S E T U P
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


        pass

    # ==================================================================================================================
    #
    #                                     C A L L B A C K  M E T H O D S  (T O P I C S)
    #
    # ==================================================================================================================

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