# ROS MODULES
import rospy
import actionlib


# Action
from crazyflie_messages.msg import TakeoffAction, TakeoffGoal, TakeoffResult, TakeoffFeedback

class CrazyPyramidSwarmSim:
    # ==================================================================================================================
    #
    #                                               C O N S T R U C T O R
    #
    # This class completely handle one virtual pyramid swarm of crazyflies.
    # INPUTS:
    #   1) cf_names -> list of names of each Crazyflie within the virtual swarm;
    #
    # ==================================================================================================================
    def __init__(self, cf_names, number_of_levels, vertical_offset):
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                           P R O P E R T I E S  I N I T I A L I Z A T I O N
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # List of crazyflies names:
        self.cf_names = cf_names

        # Number of crazyflies:
        self.number_of_cfs = len(self.cf_names)

        # Pyramid parameters:
        self.levels = number_of_levels
        self.vertical_offset = vertical_offset

        # List of clients for takeoff action per each drone:
        self.takeoff_act_clients = []
        self.__make_takeoff_clients()

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       S U B S C R I B E R S  S E T U P
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
        # Action to take off the entire swarm:
        self.__swarm_takeoff_act = actionlib.SimpleActionServer('/pyramid_swarm/takeoff_actn', TakeoffAction,
                                                                self.__swarm_takeoff_act_callback, False)
        self.__swarm_takeoff_act.start()

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

    # ==================================================================================================================
    #
    #                                 C A L L B A C K  M E T H O D S  (A C T I O N S)
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                             __S W A R M _ T A K E O F F _ A C T _ C A L L B A C K
    #
    # This method is used as the swarm takeoff action.
    # ------------------------------------------------------------------------------------------------------------------
    def __swarm_takeoff_act_callback(self, goal):
        # Output:
        feedback = TakeoffFeedback()
        result = TakeoffResult()

        # Setting up takeoff request:
        _goal = TakeoffGoal()
        _goal.takeoff_height = goal.takeoff_height

        for takeoff_actn in self.takeoff_act_clients:
            takeoff_actn.send_goal(_goal, feedback_cb=self.__cf_takeoff_feedback_cb)
            # print('\n\nTIPO:' + str(type(takeoff_actn)))
        # Vertex drone takeoff:
        takeoff_height = goal.takeoff_height + self.levels * self.vertical_offset
        _custom_goal = TakeoffGoal()
        _custom_goal.takeoff_height = takeoff_height
        self.takeoff_act_clients[0].send_goal(_custom_goal, feedback_cb=self.__cf_takeoff_feedback_cb)
        prev_level_pos = 0
        for ii in range(1, self.levels + 1):
            amount_of_cfs_per_level = 2 * (ii + 1) + 2 * (ii - 1)
            for jj in range(prev_level_pos + 1, prev_level_pos + amount_of_cfs_per_level + 1):
                level_height = takeoff_height - ii * self.vertical_offset
                _custom_goal.takeoff_height = level_height
                self.takeoff_act_clients[jj].send_goal(_custom_goal, feedback_cb=self.__cf_takeoff_feedback_cb)
            prev_level_pos = prev_level_pos + amount_of_cfs_per_level


        self.__swarm_takeoff_act.set_succeeded(result)

    # ==================================================================================================================
    #
    #                          F E E D B A C K  C A L L B A C K  M E T H O D S  (A C T I O N S)
    #
    # ==================================================================================================================
    def __cf_takeoff_feedback_cb(self, feedback):
        pass