# ROS
import rospy
import actionlib

# Generic modules
import logging
import time

# Custom modules
from crazy_common_py.dataTypes import Vector3

# Action messages:
from crazyflie_messages.msg import TakeoffAction, TakeoffGoal, TakeoffResult, TakeoffFeedback

# Crazyflie API
import cflib.crtp
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
        # Name (used for setting up topics, actions and services):
        self.cfName = name

        # URI of the crazyflie (unique address):
        self.URI = URI

        # Initial position (Vector3):
        self.__initial_position = initialPosition

        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       S U B S C R I B E R S  S E T U P
        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       P U B L I S H E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Publisher used to publish real Crazyflie state:

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                           S E R V I C E S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                           A C T I O N S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Takeoff action:
        self.__takeoff_act = actionlib.SimpleActionServer('/' + name + '/takeoff_actn', TakeoffAction,
                                                          self.__takeoff_act_callback, False)
        self.__takeoff_act.start()

        # Landing action:
        self.__land_act = actionlib.SimpleActionServer('/' + name + '/land_actn', TakeoffAction,
                                                       self.__land_act_callback, False)
        self.__land_act.start()

        # Relative velocity motion:
        self.__rel_vel_move_act = actionlib.SimpleActionServer('/' + name + '/rel_vel_move_actn', TakeoffAction,
                                                               self.__rel_vel_move_act_callback, False)
        self.__rel_vel_move_act.start()

        # Relative displacement motion;
        self.__rel_displ_move_act = actionlib.SimpleActionServer('/' + name + '/rel_displ_move_actn', TakeoffAction,
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

        self.__mc.take_off()
        #self.__scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 20000)

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
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                   T A K E O F F _ A C T _ C A L L B A C K
    #
    # Callback for takeoff action.
    # ------------------------------------------------------------------------------------------------------------------
    def __takeoff_act_callback(self, goal):
        pass

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
        pass

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
        pass

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


