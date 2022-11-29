#! /usr/bin/env python3

# ROS imports
import rospy

import numpy as np
import math

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from crazyflie_messages.msg import OpticalFlow


class OpticFlowDataManage():
    def __init__(self):
        # Initialize parameters
        self.delta_x = 0
        self.delta_y = 0
        self.dt = 0
        self.px_incre = None
        self.py_incre = None
        self.h = 0
        self.gyr_x = 0
        self.gyr_y = 0
        self.gyr_z = 0

        self.init_pos_x = 0
        self.init_pos_y = 0

        self.curr_pos_x = 0
        self.curr_pos_y = 0

        self.prev_z = 0
        self.vel_z = 0

        self.vo_msg = Odometry()
        # self.pos_z_msg = Odometry()

        self.orientation_x = 0
        self.orientation_y = 0
        self.orientation_z = 0
        self.orientation_w = 0

        #Node cycle rate
        self.rate = rospy.Rate(60)

        # Subscribe to the optical flow message
        rospy.Subscriber("/optic_flow", OpticalFlow, self.optic_flow_clbk)

        # Subscribe to the laser reading collectiion
        rospy.Subscriber("/cf1/laser_reading_tot", Float64MultiArray, self.clbk_laser)

        # Subscribe to the imu gyroscope data
        rospy.Subscriber('/cf1/imu',Imu, self.clbk_imu)
       
        # Publish the velocity in real world
        # self.optic_flow_pub_vel_x = rospy.Publisher("optic_flow_vel_x", Float64, queue_size=1)
        # self.optic_flow_pub_vel_y = rospy.Publisher("optic_flow_vel_y", Float64, queue_size=1)

        # Publish the relative movement in real world
        # self.optic_flow_pub_delta_x = rospy.Publisher("optic_flow_delta_x", Float64, queue_size=1)
        # self.optic_flow_pub_delta_y = rospy.Publisher("optic_flow_delta_y", Float64, queue_size=1)

        # Publish the current xyz positions in real world
        self.optic_flow_pub_curr_x = rospy.Publisher("optic_flow_pos_x", Float64, queue_size=1)
        self.optic_flow_pub_curr_y = rospy.Publisher("optic_flow_pos_y", Float64, queue_size=1)
        self.optic_flow_pub_curr_z = rospy.Publisher("optic_flow_pos_z", Float64, queue_size=1)

        # Publishe the vo message (for ekf, visual odometry)
        self.optic_flow_pub_pose = rospy.Publisher("/vo", Odometry, queue_size=1)

        # Publish only the bottom laser ranger data as an input to import inside ekf filter
        # self.laser_pub_pos_z = rospy.Publisher("/posz", Odometry, queue_size=1)


    def clbk_imu(self,msg):
        # get angular velocity from msg
        self.gyr_x = msg.angular_velocity.x
        self.gyr_y = msg.angular_velocity.y
        self.gyr_z = msg.angular_velocity.z
        self.orientation_x = msg.orientation.x
        self.orientation_y = msg.orientation.y
        self.orientation_z = msg.orientation.z
        self.orientation_w = msg.orientation.w

    def clbk_laser(self, msg):
        # Get laser reading from subscribed topic
        self.h = msg.data[5]
        if self.prev_z == 0:
            self.prev_z = self.h
            return
        self.vel_z = (self.h-self.prev_z)/(1/100)
        self.prev_z = self.h


        # self.pos_z_msg.header.frame_id = "crazyflie_main_body"
        # self.pos_z_msg.pose.pose.position.z = self.h
        # self.laser_pub_pos_z.publish(self.pos_z_msg)


    def optic_flow_clbk(self, msg):
        self.dt = msg.dt
        self.px_incre = msg.vx # vector in x direction
        self.py_incre = msg.vy # vector in y direction
        self.secs = msg.header.stamp.secs
        self.nsecs = msg.header.stamp.nsecs

        # if self.dt != 0:
        #     for j in self.py_incre:
        #         k = j/self.dt
        #         print(k)

    def take_action(self):
        while not rospy.is_shutdown():
            if self.dt == 0:
                # self.prev_z = self.h
                continue
            
            # calculate average pixels increment in two direction
            delta_px_mean = np.mean(self.px_incre)
            delta_py_mean = np.mean(self.py_incre)

            # calcualte real velocity, in my case the image field is opposite to world field
            theta_py = math.radians(42)
            theta_px = math.radians(42)
            h = self.h
            Nx = 350
            Ny = 350
            pitch_rate = self.gyr_y
            roll_rate = self.gyr_x
            vel_x = h*theta_py*delta_py_mean/(self.dt*Nx)-h*pitch_rate
            vel_y = h*theta_px*delta_px_mean/(self.dt*Ny)-h*roll_rate
            
            # vel_z = (h-self.prev_z)/self.dt
            # publish optic flow velocity_x and velocity_y data in real world
            # self.optic_flow_pub_vel_x.publish(vel_x)
            # self.optic_flow_pub_vel_y.publish(vel_y)

            # integral velocity to get the relatve movements
            self.delta_x += vel_x*self.dt
            self.delta_y += vel_y*self.dt

            # publish relative movements in real world
            # self.optic_flow_pub_delta_x.publish(self.delta_x)
            # self.optic_flow_pub_delta_y.publish(self.delta_y)

            # calculate the current xy positions
            self.curr_pos_x = self.init_pos_x + self.delta_x
            self.curr_pos_y = self.init_pos_y+ self.delta_y

            # publishe current xyz positions
            self.optic_flow_pub_curr_x.publish(self.curr_pos_x)
            self.optic_flow_pub_curr_y.publish(self.curr_pos_y)
            self.optic_flow_pub_curr_z.publish(self.h)

            # fill the vo message

            self.vo_msg.header.frame_id = "crazyflie_main_body"
            self.current_time = rospy.get_rostime()
            self.vo_msg.header.stamp = self.current_time
            
            # self.vo_msg.header.stamp.secs = self.secs
            # self.vo_msg.header.stamp.nsecs = self.nsecs
            # self.vo_msg.child_frame_id = "crazyflie_main_body"

            self.vo_msg.pose.pose.position.x = self.curr_pos_x
            self.vo_msg.pose.pose.position.y = self.curr_pos_y
            self.vo_msg.pose.pose.position.z = self.h
            
            self.vo_msg.pose.pose.orientation.x = self.orientation_x
            self.vo_msg.pose.pose.orientation.y = self.orientation_y
            self.vo_msg.pose.pose.orientation.z = self.orientation_z
            self.vo_msg.pose.pose.orientation.w = self.orientation_w

            # self.vo_msg.twist.twist.linear.x = vel_x
            # self.vo_msg.twist.twist.linear.y = vel_y
            # self.vo_msg.twist.twist.linear.z = self.vel_z

            # self.vo_msg.twist.twist.angular.x = self.gyr_x
            # self.vo_msg.twist.twist.angular.y = self.gyr_y
            # self.vo_msg.twist.twist.angular.z = self.gyr_z


            self.vo_msg.pose.covariance = [0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 10000, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 10000, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0, 10000]
            
            # self.vo_msg.twist.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 
            #                                 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 
            #                                 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 
            #                                 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 
            #                                 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 
            #                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]

            # publish the vo message
            self.optic_flow_pub_pose.publish(self.vo_msg)
            # self.prev_z = h
            self.rate.sleep()

if __name__ == '__main__': 
    rospy.init_node('optic_flow_data_manage', anonymous=True)
    optic_flow_data_manage = OpticFlowDataManage()
    optic_flow_data_manage.take_action()



