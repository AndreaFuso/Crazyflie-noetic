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
from std_msgs.msg import Empty
from std_msgs.msg import Int64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from crazyflie_messages.msg import CrazyflieState

class FakeStateEstimator():
    # ==================================================================================================================
    #                                               C O N S T R U C T O R
    #
    # INPUTS:
    #   1) cfName -> name of the Crazyflie in the simulation;
    #
    # ==================================================================================================================
    def __init__(self, cfName):

        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                           P R O P E R T I E S  I N I T I A L I Z A T I O N
        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        # Crazyflie name:
        self.cfName = cfName

        # Initialize specific state parameters for complementary filter
        self.prevRoll_cf = 0.0
        self.prevPitch_cf = 0.0
        self.prevYaw_cf = 0.0

        # Initialize for EKF
        self.prevRoll_ekf = 0.0
        self.prevPitch_ekf = 0.0
        self.prevYaw_ekf = 0.0

        # Setting default values for rotation:
        self.prevRoll = 0.0
        self.prevPitch = 0.0
        self.prevYaw = 0.0

        # Setting default values for velocity
        self.prevPose_x = 0.0
        self.prevPose_y = 0.0
        self.prevPose_z = 0.0

        # Setting last time instant is zero
        self.last_time = 0.0

        # Check variable to understand if the values have been correctly initialized:
        self.prev_values_init = False
        self.prev_values_init_cf = False
        self.prev_values_init_ekf = False

        # Actual_state(ground truth)
        self.__actual_state = CrazyflieState()
        self.state_comple_filter = CrazyflieState()
        self.state_ekf = CrazyflieState()

        # Initialize the filter flag
        self.filter_flag = 0
        
        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       S U B S C R I B E R S  S E T U P
        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        # Filter flag subscriber
        self.filter_flag_sub = rospy.Subscriber('/filter_flag', Int64, self.__filter_flag_sub_callback)

        # Ground truth subscriber
        self.gazebo_odom_sub = rospy.Subscriber('/' + cfName + '/' + DEFAULT_ODOMETRY_TOPIC,
                                                Odometry, self.__gazebo_odom_sub_callback)

        # Subscribers: complementary filter and optical flow subscriber
        self.imu_comple_sub = rospy.Subscriber('/imu/data',Imu,self.__imu_comple_sub_callback)
        
        # Subscribers: EKF results subscriber
        # self.ekf_sub = rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.__ekf_sub_callback)
        self.ekf_sub = rospy.Subscriber('/odometry/visual', Odometry, self.__ekf_sub_callback)
        # Subscribe to a pace_500Hz topic to publish estimated states
        self.pace_500_sub = rospy.Subscriber('/pace_500Hz',Empty, self.__pace_500_sub_callback)


        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       P U B L I S H E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        self.state_pub = rospy.Publisher('/' + cfName + '/' + DEFAULT_CF_STATE_TOPIC, CrazyflieState, queue_size=1)
        self.state_comple_filter_pub = rospy.Publisher("/state_comple", CrazyflieState, queue_size=1)
        self.state_ekf_pub = rospy.Publisher("/state_ekf", CrazyflieState, queue_size=1)

        self.filter_flag_pub = rospy.Publisher('/filter_flag', Int64, queue_size=1)


    def __filter_flag_sub_callback(self,msg):
        data = msg.data
        if data == None:
            self.filter_flag = 0
        else:
            self.filter_flag = data

    def __pace_500_sub_callback(self,msg):
        if self.filter_flag == 0:
            self.state_pub.publish(self.__actual_state)
            self.state_ekf_pub.publish(self.state_ekf)
        elif self.filter_flag == 1:
            self.state_pub.publish(self.state_comple_filter)
            # self.state_pub.publish(self.__actual_state)
            self.state_comple_filter_pub.publish(self.state_comple_filter)
        elif self.filter_flag == 2:
            self.state_pub.publish(self.state_ekf)
            self.state_pub.publish(self.__actual_state)
            self.state_ekf_pub.publish(self.state_ekf)
            # self.state_ekf_pub.publish(self.__actual_state)

######################################################################################

#              Ground Truth

######################################################################################
  
    def __gazebo_odom_sub_callback(self, msg):        
        # Setting the name:
        self.__actual_state.name = self.cfName

        # if self.filter_flag == 0:
        # Setting the position:
        self.__actual_state.position.x = msg.pose.pose.position.x
        self.__actual_state.position.y = msg.pose.pose.position.y
        self.__actual_state.position.z = msg.pose.pose.position.z

        # Setting the orientation (radians):
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

        # Setting the angular velocity from Gazebo
        # self.__actual_state.rotating_speed.x = msg.twist.twist.angular.x
        # self.__actual_state.rotating_speed.y = msg.twist.twist.angular.y
        # self.__actual_state.rotating_speed.z = msg.twist.twist.angular.z

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
        else:
            self.__actual_state.rotating_speed.x = 0.0
            self.__actual_state.rotating_speed.y = 0.0
            self.__actual_state.rotating_speed.z = 0.0

        self.prev_values_init = True
    
        # elif self.filter_flag == 2:
            # self.__actual_state.position.x = self.state_ekf.position.x
            # self.__actual_state.position.y = self.state_ekf.position.y
            # self.__actual_state.position.z = self.state_ekf.position.z

            # self.__actual_state.orientation.roll = self.state_ekf.orientation.roll
            # self.__actual_state.orientation.pitch = self.state_ekf.orientation.pitch
            # self.__actual_state.orientation.yaw = self.state_ekf.orientation.yaw

            # self.__actual_state.velocity.x = self.state_ekf.velocity.x
            # self.__actual_state.velocity.y = self.state_ekf.velocity.y
            # self.__actual_state.velocity.z = self.state_ekf.velocity.z

            # self.__actual_state.rotating_speed.x = self.state_ekf.rotating_speed.x
            # self.__actual_state.rotating_speed.y = self.state_ekf.rotating_speed.y
            # self.__actual_state.rotating_speed.z = self.state_ekf.rotating_speed.z

######################################################################################

#            Complementary Filter Application(only roll pitch yaw)

######################################################################################

    def __imu_comple_sub_callback(self, msg):
    
        # Setting the name:
        self.state_comple_filter.name = self.cfName

        # Setting the orientation(radians):
        (roll_cf, pitch_cf, yaw_cf) = tf.transformations.euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z,
                msg.orientation.w])
        
        self.state_comple_filter.orientation.roll = roll_cf
        self.state_comple_filter.orientation.pitch = pitch_cf
        self.state_comple_filter.orientation.yaw = yaw_cf

        # Setting the position using ground truth
        self.state_comple_filter.position.x = self.__actual_state.position.x
        self.state_comple_filter.position.y = self.__actual_state.position.y
        self.state_comple_filter.position.z = self.__actual_state.position.z

        # Setting the velocity using ground truth
        self.state_comple_filter.velocity.x = self.__actual_state.velocity.x
        self.state_comple_filter.velocity.y = self.__actual_state.velocity.y
        self.state_comple_filter.velocity.z = self.__actual_state.velocity.z

        if self.prev_values_init_cf:
            # Setting the angular velocity:
            dt = 1 / 100
            self.state_comple_filter.rotating_speed.x = (roll_cf - self.prevRoll_cf) / dt
            self.state_comple_filter.rotating_speed.y = (pitch_cf - self.prevPitch_cf) / dt
            self.state_comple_filter.rotating_speed.z = (yaw_cf - self.prevYaw_cf) / dt

            self.prevRoll_cf = roll_cf
            self.prevPitch_cf = pitch_cf
            self.prevYaw_cf = yaw_cf

        else:
            self.state_comple_filter.rotating_speed.x = 0.0
            self.state_comple_filter.rotating_speed.y = 0.0
            self.state_comple_filter.rotating_speed.z = 0.0

        self.prev_values_init_cf = True


#################################################################################### 

#              Extended Kalman Filter

####################################################################################

    def __ekf_sub_callback(self, msg):
        self.state_ekf.name = self.cfName

        self.state_ekf.position.x = msg.pose.pose.position.x
        self.state_ekf.position.y = msg.pose.pose.position.y
        self.state_ekf.position.z = msg.pose.pose.position.z

        (roll_ekf, pitch_ekf, yaw_ekf) = tf.transformations.euler_from_quaternion(
            [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w])
        self.state_ekf.orientation.roll = roll_ekf
        self.state_ekf.orientation.pitch = pitch_ekf
        self.state_ekf.orientation.yaw = yaw_ekf

        self.state_ekf.velocity.x = msg.twist.twist.linear.x
        self.state_ekf.velocity.y = msg.twist.twist.linear.y
        self.state_ekf.velocity.z = msg.twist.twist.linear.z

        self.state_ekf.rotating_speed.x = msg.twist.twist.angular.x
        self.state_ekf.rotating_speed.y = msg.twist.twist.angular.y
        self.state_ekf.rotating_speed.z = msg.twist.twist.angular.z

        # if self.prev_values_init_ekf:
        #     # Setting the angular velocity:
        #     dt = 1 / 500
        #     self.state_ekf.rotating_speed.x = (roll_ekf - self.prevRoll_ekf) / dt
        #     self.state_ekf.rotating_speed.y = (pitch_ekf - self.prevPitch_ekf) / dt
        #     #self.__actual_state.rotating_speed.z = msg.twist.twist.angular.z
        #     self.state_ekf.rotating_speed.z = (yaw_ekf - self.prevYaw_ekf) / dt

        #     self.prevRoll = roll_ekf
        #     self.prevPitch = pitch_ekf
        #     self.prevYaw = yaw_ekf
        # else:
        #     self.state_ekf.rotating_speed.x = 0.0
        #     self.state_ekf.rotating_speed.y = 0.0
        #     self.state_ekf.rotating_speed.z = 0.0

        # self.prev_values_init_ekf = True
       
        
'''
        secs = msg.header.stamp.secs
        nsecs = msg.header.stamp.nsecs
        curr_time = float(secs) + float(nsecs)*1e-9

        if self.last_time == 0:
            
            self.last_time = curr_time

            self.prevRoll_ekf = roll_ekf
            self.prevPitch_ekf = pitch_ekf
            self.prevYaw_ekf = yaw_ekf

            self.prevPose_x = self.state_ekf.position.x
            self.prevPose_y = self.state_ekf.position.y
            self.prevPose_z = self.state_ekf.position.z
            
            return
        
        dt = curr_time - self.last_time

        self.state_ekf.rotating_speed.x = (roll_ekf - self.prevRoll_ekf) / dt
        self.state_ekf.rotating_speed.y = (pitch_ekf - self.prevPitch_ekf) / dt
        self.state_ekf.rotating_speed.z = (yaw_ekf - self.prevYaw_ekf) / dt

        self.prevRoll_ekf = roll_ekf
        self.prevPitch_ekf = pitch_ekf
        self.prevYaw_ekf = yaw_ekf

        # Setting the linear velocity
        self.state_ekf.velocity.x = (self.state_ekf.position.x - self.prevPose_x) / dt
        self.state_ekf.velocity.y = (self.state_ekf.position.y - self.prevPose_y) / dt
        self.state_ekf.velocity.z = (self.state_ekf.position.z - self.prevPose_z) / dt

        self.prevPose_x = self.state_ekf.position.x
        self.prevPose_y = self.state_ekf.position.y
        self.prevPose_z = self.state_ekf.position.z

        self.last_time = curr_time        
        # self.prev_values_init_ekf = True 
'''