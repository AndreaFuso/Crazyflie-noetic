# ROS MODULES
import math

import rospy
import tf

# CUSTOM MODULES
from crazy_common_py.common_functions import quat2euler, RotateVector, rad2deg
from crazy_common_py.dataTypes import Vector3
from crazy_common_py.default_topics import DEFAULT_ODOMETRY_TOPIC, DEFAULT_CF_STATE_TOPIC

# MESSAGES
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from crazyflie_messages.msg import CrazyflieState


class FakeStateEstimator:
    # ==================================================================================================================
    #
    #                                               C O N S T R U C T O R
    #
    # INPUTS:
    #   1) cfName -> name of the Crazyflie in the simulation;
    #
    # ==================================================================================================================
    def __init__(self, cfName):
        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                               W A I T I N G  F O R  S E R V I C E S
        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                           P R O P E R T I E S  I N I T I A L I Z A T I O N
        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Crazyflie name:
        self.cfName = cfName

        # Setting default values for rotation:
        self.prevRoll = 0.0
        self.prevPitch = 0.0
        self.prevYaw = 0.0

        # Check variable to understand if the values have been correctly initialized:
        self.prev_values_init = False

        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       S U B S C R I B E R S  S E T U P
        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #gazebo_imu_sub = rospy.Subscriber('/' + cfName + '/imu', Imu, self.__gazebo_imu_sub_callback)
        self.gazebo_odom_sub = rospy.Subscriber('/' + cfName + '/' + DEFAULT_ODOMETRY_TOPIC,
                                                Odometry, self.__gazebo_odom_sub_callback)

        #self.gazebo_odom_REL_sub = rospy.Subscriber('/' + cfName + '/odom_relative', Odometry,
        #                                        self.__gazebo_odom_sub_REL_callback)
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       P U B L I S H E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        self.state_pub = rospy.Publisher('/' + cfName + '/' + DEFAULT_CF_STATE_TOPIC, CrazyflieState, queue_size=1)
        self.__actual_state = CrazyflieState()
        self.__tmp_rotating_speed = Vector3()


        #self.state_pubREL = rospy.Publisher('/' + cfName + '/state_rel', CrazyflieState, queue_size=1)
        #self.imu_pub = rospy.Publisher('/' + cfName + '/state_imu', CrazyflieState, queue_size=1)
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                           S E R V I C E S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                           A C T I O N S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        # ---------------------------------------------------------------------------------------------------------------
        #                                       I N I T I A L  O P E R A T I O N S
        # ---------------------------------------------------------------------------------------------------------------


    '''def __gazebo_imu_sub_callback(self, msg):
        actual_state = CrazyflieState()
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z,
             msg.orientation.w])
        actual_state.orientation.roll = roll
        actual_state.orientation.pitch = pitch
        actual_state.orientation.yaw = yaw

        self.__tmp_rotating_speed.x = msg.angular_velocity.x
        self.__tmp_rotating_speed.y = msg.angular_velocity.y
        self.__tmp_rotating_speed.z = msg.angular_velocity.z

        self.imu_pub.publish(actual_state)'''

    '''def __gazebo_odom_sub_REL_callback(self, msg):
        actual_state = CrazyflieState()
        # Setting the position:
        actual_state.position.x = msg.pose.pose.position.x
        actual_state.position.y = msg.pose.pose.position.y
        actual_state.position.z = msg.pose.pose.position.z

        # Setting the orientation:
        # rpy = quat2euler(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
        # msg.pose.pose.orientation.w)
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w])
        actual_state.orientation.roll = roll
        actual_state.orientation.pitch = pitch
        actual_state.orientation.yaw = yaw

        # Setting the linear velocity:
        actual_state.velocity.x = msg.twist.twist.linear.x
        actual_state.velocity.y = msg.twist.twist.linear.y
        actual_state.velocity.z = msg.twist.twist.linear.z

        actual_state.rotating_speed.x = msg.twist.twist.angular.x
        actual_state.rotating_speed.y = msg.twist.twist.angular.y
        actual_state.rotating_speed.z = msg.twist.twist.angular.z

        self.state_pubREL.publish(actual_state)'''

    def __gazebo_odom_sub_callback(self, msg):
        # Setting the name:
        self.__actual_state.name = self.cfName

        # Setting the position:
        self.__actual_state.position.x = msg.pose.pose.position.x
        self.__actual_state.position.y = msg.pose.pose.position.y
        self.__actual_state.position.z = msg.pose.pose.position.z

        # Setting the orientation (radians):
        #rpy = quat2euler(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                         #msg.pose.pose.orientation.w)
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w])
        self.__actual_state.orientation.roll = roll
        self.__actual_state.orientation.pitch = pitch
        self.__actual_state.orientation.yaw = yaw

        # Setting the linear velocity:
        self.__actual_state.velocity.x = msg.twist.twist.linear.x
        self.__actual_state.velocity.y = msg.twist.twist.linear.y
        self.__actual_state.velocity.z = msg.twist.twist.linear.z

        if self.prev_values_init:
            # Setting the angular velocity:
            dt = 1 / 500
            self.__actual_state.rotating_speed.x = (roll - self.prevRoll) / dt
            self.__actual_state.rotating_speed.y = (pitch - self.prevPitch) / dt
            #self.__actual_state.rotating_speed.z = msg.twist.twist.angular.z
            self.__actual_state.rotating_speed.z = (yaw - self.prevYaw) / dt



            self.prevRoll = roll
            self.prevPitch = pitch
            self.prevYaw = yaw
            '''abs_rr1 = self.__actual_state.rotating_speed.x + math.sin(roll) * math.tan(pitch) * self.__actual_state.rotating_speed.y + math.cos(roll) * math.tan(pitch) * self.__actual_state.rotating_speed.z
            abs_pr1 = math.cos(roll) * self.__actual_state.rotating_speed.y - math.sin(roll) * self.__actual_state.rotating_speed.z
            abs_yr1 = (math.sin(roll) / math.cos(pitch)) * self.__actual_state.rotating_speed.y + (math.cos(roll) / math.cos(pitch)) * self.__actual_state.rotating_speed.z

            abs_rr2 = msg.twist.twist.angular.x + math.sin(roll) * math.tan(pitch) * msg.twist.twist.angular.y + math.cos(roll) * math.tan(pitch) * msg.twist.twist.angular.z
            abs_pr2 = math.cos(roll) * msg.twist.twist.angular.y - math.sin(roll) * msg.twist.twist.angular.z
            abs_yr2 = (math.sin(roll) / math.cos(pitch)) * msg.twist.twist.angular.y + (math.cos(roll) / math.cos(pitch)) * msg.twist.twist.angular.z

            print('\nAngular pos: [', rad2deg(roll), ', ', rad2deg(pitch), ', ', rad2deg(yaw), ']')
            print('Estimated: [', rad2deg(self.__actual_state.rotating_speed.x), ';', rad2deg(self.__actual_state.rotating_speed.y), ';', rad2deg(self.__actual_state.rotating_speed.z), ']')
            print('Gazebo: [', rad2deg(msg.twist.twist.angular.x), ';', rad2deg(msg.twist.twist.angular.y), ';', rad2deg(msg.twist.twist.angular.z), ']')
            print('Calculated1: [', rad2deg(abs_rr1), ';', rad2deg(abs_pr1), ';', rad2deg(abs_yr1), ']')
            print('Calculated2: [', rad2deg(abs_rr2), ';', rad2deg(abs_pr2), ';', rad2deg(abs_yr2), ']\n')

            self.__actual_state.rotating_speed.x = abs_rr2
            self.__actual_state.rotating_speed.y = abs_pr2
            self.__actual_state.rotating_speed.z = abs_yr2'''

        else:
            self.__actual_state.rotating_speed.x = 0.0
            self.__actual_state.rotating_speed.y = 0.0
            self.__actual_state.rotating_speed.z = 0.0

        self.prev_values_init = True

        # Publishing the state:
        self.state_pub.publish(self.__actual_state)