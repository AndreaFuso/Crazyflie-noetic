#! /usr/bin/env python3

# in this script, we will write python codes to double integral the acceleration to get the relative movements
# and then get the xy coordinates in real world

# ROS MODULES
import rospy

# Other modules
import math

# MESSAGES
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64 
from std_msgs.msg import Float64MultiArray

class PosFromImu():
    def __init__(self):
        # list some parameters
        self.updateRate =  100 #Hz
        self.dt = 1/self.updateRate #s
        
        self.vel_x = 0
        self.delta_x = 0
        self.init_pos_x = 0

        self.vel_y = 0
        self.delta_y = 0
        self.init_pos_y = 0

        self.vel_z = 0
        self.delta_z = 0
        self.init_pos_z = 0.5

        self.bottom = 0

        # initialize linear acceleration
        self.acc_x, self.acc_y, self.acc_z = 0, 0, 0
        
        self.rate = rospy.Rate(100)

        # Subscribers
        rospy.Subscriber("/cf1/laser_reading_tot", Float64MultiArray, self.clbk_laser)
        rospy.Subscriber('/cf1/imu',Imu, self.clbk_imu)
        
        # Publishers 
        self.pos_x_from_imu_pub = rospy.Publisher("imu_pos_x", Float64, queue_size=1)
        self.pos_y_from_imu_pub = rospy.Publisher("imu_pos_y", Float64, queue_size=1)
        # self.pos_z_from_imu_pub = rospy.Publisher("imu_pos_z", Float64, queue_size=1)
    
    def clbk_laser(self, msg):
        # Get laser reading from subscribed topic
        self.bottom = msg.data[5]

    def clbk_imu(self,msg):
        # get accleration form msg
        self.acc_x = msg.linear_acceleration.x
        self.acc_y = msg.linear_acceleration.y
        self.acc_z = msg.linear_acceleration.z 

    def take_action_publish_pos(self):
        while not rospy.is_shutdown():
            # Get the initial z position from laser reading
            self.init_pos_z = self.bottom
            
            # Double integration for the acceleration
            self.vel_x = self.vel_x + self.acc_x*self.dt
            self.delta_x += self.vel_x*self.dt
            self.curr_pos_x = self.init_pos_x + self.delta_x

            self.vel_y = self.vel_y + self.acc_y*self.dt
            self.delta_y += self.vel_y*self.dt
            self.curr_pos_y = self.init_pos_y + self.delta_y

            # self.vel_z = self.vel_z + (self.acc_z-9.8)*self.dt
            # self.delta_z += self.vel_z*self.dt
            # self.curr_pos_z = self.init_pos_z + self.delta_z

            # Publish the xyz positions
            self.pos_x_from_imu_pub.publish(self.curr_pos_x)
            self.pos_y_from_imu_pub.publish(self.curr_pos_y)
            # self.pos_z_from_imu_pub.publish(self.curr_pos_z)

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node("pos_from_imu")
    my_node = PosFromImu()
    my_node.take_action_publish_pos()
