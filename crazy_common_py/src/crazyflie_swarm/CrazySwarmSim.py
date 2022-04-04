# ROS MODULES
import rospy
import actionlib


# Action
from crazyflie_messages.msg import TakeoffAction, TakeoffGoal, TakeoffResult, TakeoffFeedback
from crazyflie_messages.msg import EmptyAction, EmptyGoal, EmptyResult, EmptyFeedback
from crazy_common_py.default_topics import DEFAULT_FLOCK_TOPIC, DEFAULT_CF_STATE_TOPIC, DEFAULT_100Hz_PACE_TOPIC
from crazy_common_py.constants import DEFAULT_LEADER
from std_msgs.msg import Empty
from crazyflie_messages.msg import Position, CrazyflieState, SwarmStates

from crazy_common_py.common_functions import deg2rad, extractCfNumber
from crazy_common_py.constants import *

class CrazySwarmSim:
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
        # List of crazyflies names:
        self.cf_names = cf_names

        # Number of crazyflies:
        self.number_of_cfs = len(self.cf_names)

        # List of clients for takeoff action per each drone:
        self.takeoff_act_clients = []
        self.__make_takeoff_clients()

        # List of clients for flocking action:
        self.flocking_act_clients = []
        self.__make_flocking_clients()

        # List of state subscribers:
        self.state_subs = []
        self.states = []
        self.__make_state_subs()

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       S U B S C R I B E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Subscriber to pace 100Hz:
        self.pace_100Hz_sub = rospy.Subscriber('/' + DEFAULT_100Hz_PACE_TOPIC, Empty, self.__pace_100Hz_sub_callback)
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       P U B L I S H E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # States publisher:
        self.states_pub = rospy.Publisher('/swarm/states', SwarmStates, queue_size=1)
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

        self.__swarm_flocking_act = actionlib.SimpleActionServer('/swarm/flocking_actn', EmptyAction,
                                                                 self.__swarm_flocking_act_callback, False)
        self.__swarm_flocking_act.start()

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

    def __make_flocking_clients(self):
        for cf_name in self.cf_names:
            if cf_name != DEFAULT_LEADER:
                tmp_action = actionlib.SimpleActionClient('/' + cf_name + '/' + DEFAULT_FLOCK_TOPIC, EmptyAction)
                self.flocking_act_clients.append(tmp_action)
                self.flocking_act_clients[-1].wait_for_server()

    def __make_state_subs(self):
        for cf_name in self.cf_names:
            tmp_sub = rospy.Subscriber('/' + cf_name + '/' + DEFAULT_CF_STATE_TOPIC, CrazyflieState, self.__state_cb)
            self.state_subs.append(tmp_sub)
            self.states.append(CrazyflieState())

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
        # Initializing states message:
        states = SwarmStates()
        states.states = self.states
        self.states_pub.publish(states)
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
            #print('\n\nTIPO:' + str(type(takeoff_actn)))

        self.__swarm_takeoff_act.set_succeeded(result)

    def __swarm_flocking_act_callback(self, goal):
        # Defining the goal:
        flock_goal = EmptyGoal()

        name = []
        # Sending the goal to all followers clients:
        for ii in range(0, len(self.flocking_act_clients)):
            self.flocking_act_clients[ii].send_goal(flock_goal)

        # Sending result:
        result = EmptyResult()
        result.executed = True

        self.__swarm_flocking_act.set_succeeded(result)
    # ==================================================================================================================
    #
    #                          F E E D B A C K  C A L L B A C K  M E T H O D S  (A C T I O N S)
    #
    # ==================================================================================================================
    def __cf_takeoff_feedback_cb(self, feedback):
        pass