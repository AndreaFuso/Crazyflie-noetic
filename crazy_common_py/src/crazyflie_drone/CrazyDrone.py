# ROS
import rospy
import actionlib

# Generic modules
import logging
import time

# Custom modules
from crazy_common_py.dataTypes import Vector3
from crazy_common_py.default_topics import DEFAULT_TAKEOFF_ACT_TOPIC, DEFAULT_LAND_ACT_TOPIC, DEFAULT_REL_VEL_TOPIC, \
    DEFAULT_REL_POS_TOPIC, DEFAULT_CF_STATE_TOPIC, DEFAULT_100Hz_PACE_TOPIC
from crazy_common_py.common_functions import deg2rad
# Action messages:
from crazyflie_messages.msg import TakeoffAction, TakeoffGoal, TakeoffResult, TakeoffFeedback
from crazyflie_messages.msg import Destination3DAction, Destination3DGoal, Destination3DFeedback
from crazyflie_messages.msg import CrazyflieState
from std_msgs.msg import Empty

# Crazyflie API
import cflib.crtp
from cflib.crazyflie.syncLogger import SyncLogger
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

        # Logger:
        self.__attitude_logger = LogConfig(name='attitude_conf', period_in_ms=10)
        self.__attitude_logger.add_variable('stabilizer.roll', 'float')
        self.__attitude_logger.add_variable('stabilizer.pitch', 'float')
        self.__attitude_logger.add_variable('stabilizer.yaw', 'float')

        # State:
        self.__state = CrazyflieState()

        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       S U B S C R I B E R S  S E T U P
        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        self.__pace_100Hz_sub = rospy.Subscriber('/' + DEFAULT_100Hz_PACE_TOPIC, Empty, self.__pace_100Hz_cb)
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       P U B L I S H E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Publisher used to publish real Crazyflie state:
        self.__state_pub = rospy.Publisher('/' + self.cfName + '/' + DEFAULT_CF_STATE_TOPIC, CrazyflieState,
                                           queue_size=1)
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                           S E R V I C E S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                           A C T I O N S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Takeoff action:
        self.__takeoff_act = actionlib.SimpleActionServer('/' + name + '/' + DEFAULT_TAKEOFF_ACT_TOPIC,
                                                          TakeoffAction, self.__takeoff_act_callback, False)
        self.__takeoff_act.start()

        # Landing action:
        self.__land_act = actionlib.SimpleActionServer('/' + name + '/' + DEFAULT_LAND_ACT_TOPIC,
                                                       TakeoffAction, self.__land_act_callback, False)
        self.__land_act.start()

        # Relative velocity motion:
        self.__rel_vel_move_act = actionlib.SimpleActionServer('/' + name + '/' + DEFAULT_REL_VEL_TOPIC,
                                                               Destination3DAction,
                                                               self.__rel_vel_move_act_callback, False)
        self.__rel_vel_move_act.start()

        # Relative displacement motion;
        self.__rel_displ_move_act = actionlib.SimpleActionServer('/' + name + '/' + DEFAULT_REL_POS_TOPIC,
                                                                 Destination3DAction,
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

        # Logger:
        self.__logger = SyncLogger(self.__scf, self.__attitude_logger)
        self.__logger.connect()


        #self.__mc.take_off()
        #self.__scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 20000)

    # ==================================================================================================================
    #
    #                                     C A L L B A C K  M E T H O D S  (T O P I C S)
    #
    # ==================================================================================================================
    def __pace_100Hz_cb(self,msg):
        if self.__logger.is_connected():
            # Getting data from logger
            data = self.__logger.next()

            # Formulating state:
            self.__state.orientation.roll = deg2rad(data[1]['stabilizer.roll'])
            self.__state.orientation.pitch = deg2rad(- data[1]['stabilizer.pitch'])
            self.__state.orientation.yaw = deg2rad(data[1]['stabilizer.yaw'])
    
            # Publishing the state:
            self.__state_pub.publish(self.__state)
            #print(data)
            #print('ROLL: ', data[1]['stabilizer.roll'], '; PITCH: ', data[1]['stabilizer.pitch'], '; YAW: ', data[1]['stabilizer.yaw'])

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
        self.__mc.take_off()

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
        self.__mc.land()

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

        success = False

        # Getting request parameters;
        vx_des = goal.destination_info.desired_velocity.x
        vy_des = goal.destination_info.desired_velocity.y
        vz_des = goal.destination_info.desired_velocity.z
        yaw_rate_des = - goal.destination_info.desired_yaw

        time_duration = goal.time_duration

        if time_duration <= 0:
            time_duration = 1.0

        # Getting initial time and time duration (used for fixed time duration request):
        t0 = rospy.get_rostime()
        ros_time_duration = rospy.Duration.from_sec(time_duration)
        final_rostime = t0 + ros_time_duration

        feedback = Destination3DFeedback()

        # Starting the motion:
        self.__mc.start_linear_motion(vx_des, vy_des, vz_des, yaw_rate_des)

        # Wait time duration:
        while True:
            # Getting current time:
            actual_time = rospy.get_rostime()

            # Remaining time:
            remaining_time = (final_rostime - actual_time).to_sec()

            # Publishing absolute distance as feedback:
            feedback.remaining_time = remaining_time
            self.__rel_vel_move_act.publish_feedback(feedback)

            # If time duration has elapsed exit the cycle:
            if remaining_time <= 0:
                success = True
                break

        # Once time duration elapsed let's stop the crazyflie:
        self.__mc.stop()



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

        # Stop logger:
        self.__logger.disconnect()

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


