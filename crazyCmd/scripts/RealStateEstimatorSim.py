#! /usr/bin/env python3

# ROS MODULES
import rospy
import tf

# Messages
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from crazyflie_messages.msg import CrazyflieState

class RealStateEstimator():
    def __init__(self):

        # Crazyflie name:
        self.cfName = 'cf1'

        # Initialize specific state parameters
        self.position_x = 0
        self.position_y = 0
        self.position_z = 0

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.velocity_x = 0
        self.velocity_y = 0
        self.velocity_z = 0

        self.rotating_speed_x = 0
        self.rotating_speed_y = 0
        self.rotating_speed_z = 0

        # Setting default values for rotation:
        self.prevRoll = 0.0
        self.prevPitch = 0.0
        self.prevYaw = 0.0

        # Node cycle rate
        self.rate = rospy.Rate(500)

        # Initialize total state message
        self.state_cffilter_imu_optic = CrazyflieState()

        # Subscribers: complementary filter and optical flow subscriber
        self.imu_comple_sub = rospy.Subscriber('/imu/data',Imu,self.__imu_comple_sub_callback)
        self.optic_comple_sub = rospy.Subscriber('/vo',Odometry,self.__optic_comple_sub_callback)

        # Publisher: complementary filter and optical flow publisher
        self.imu_optic_comple_pub = rospy.Publisher("/cf1/state_imu_cffilter", CrazyflieState, queue_size=1)


    def __imu_comple_sub_callback(self, msg):
    
        # Setting the name:
        # self.state_cffilter_imu_optic.name = self.cfName

        # Setting the orientation(radians):
        (self.roll, self.pitch, self.yaw) = tf.transformations.euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z,
                msg.orientation.w])

        if self.prev_values_init:
            # Setting the angular velocity:
            dt = 1 / 500
            self.rotating_speed_x = (self.roll - self.prevRoll) / dt
            self.rotating_speed_y = (self.pitch - self.prevPitch) / dt
            self.rotating_speed_z = (self.yaw - self.prevYaw) / dt

            self.prevRoll = self.roll
            self.prevPitch = self.pitch
            self.prevYaw = self.yaw

        else:
            self.rotating_speed_x = 0.0
            self.rotating_speed_y = 0.0
            self.rotating_speed_z = 0.0


    def __optic_comple_sub_callback(self, msg):
        # Setting the position:
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        self.position_z = msg.pose.pose.position.z

        # Setting the linear velocity:
        self.velocity_x = msg.twist.twist.linear.x
        self.velocity_y = msg.twist.twist.linear.y
        self.velocity_z = msg.twist.twist.linear.z


    def take_action_real_state_estimator(self):        
        
        while not rospy.is_shutdown():
            self.prev_values_init = True
        # while self.prev_values_init:
            if self.position_x==0 and self.position_y==0 and self.position_z==0 and self.roll==0 and self.pitch==0 and self.yaw==0 and self.velocity_x==0 and self.velocity_y==0 and self.velocity_z==0 and self.rotating_speed_x==0 and self.rotating_speed_y==0 and self.rotating_speed_y==0 and self.rotating_speed_z==0:
                continue

            self.state_cffilter_imu_optic.name = self.cfName
            self.state_cffilter_imu_optic.position.x = self.position_x
            self.state_cffilter_imu_optic.position.y = self.position_y
            self.state_cffilter_imu_optic.position.z = self.position_z

            self.state_cffilter_imu_optic.orientation.roll = self.roll
            self.state_cffilter_imu_optic.orientation.pitch = self.pitch
            self.state_cffilter_imu_optic.orientation.yaw = self.yaw

            self.state_cffilter_imu_optic.velocity.x = self.velocity_x
            self.state_cffilter_imu_optic.velocity.y = self.velocity_y
            self.state_cffilter_imu_optic.velocity.z = self.velocity_z

            self.state_cffilter_imu_optic.rotating_speed.x = self.rotating_speed_x
            self.state_cffilter_imu_optic.rotating_speed.y = self.rotating_speed_y
            self.state_cffilter_imu_optic.rotating_speed.z = self.rotating_speed_z

            self.imu_optic_comple_pub.publish(self.state_cffilter_imu_optic)

            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('real_state_estimator')
    my_node = RealStateEstimator()
    my_node.take_action_real_state_estimator()