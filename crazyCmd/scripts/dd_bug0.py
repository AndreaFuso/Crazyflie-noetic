#! /usr/bin/env python3

# Here is the Bug0 algorithm, which drives the robot towards a points (goal),
# while doing so if the robot detects an obstacle it goes around it

# This one is included inside the launch file, but we need to takeoff and run the laser_reading_collection before

import rospy

from std_msgs.msg import Float64MultiArray
from crazyflie_messages.msg import Position
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_srvs.srv import *
from tf import transformations

import math


class Bug0():
    def __init__(self):
        # Initialize parameters
        self.front, self.back, self.left, self.right, self.top, self.bottom = 0,0,0,0,0,0
        self.odom_yaw_ = 0
        self.yaw_error_allowed_ = 5*(math.pi/180) # +/- 5 degrees
        self.odom_position_ = Point()
        self.angle = 0

        self.desired_position_ = Point()
        self.desired_position_.x = rospy.get_param('des_pos_x')
        self.desired_position_.y = rospy.get_param('des_pos_y')
        self.desired_position_.z = 0.5

        self.desired_yaw = 0
        self.err_yaw = 0

        # Initialize service clients
        self.srv_client_go_to_point_ = None
        self.srv_client_wall_follower_ = None
        
        # Machine state
        self.state_ = 0 
        # 0 - go to point
        # 1 - wall following

        self.state_disc_ = ['Go to point', 'Wall following']
        

        #Node cycle rate
        self.rate = rospy.Rate(100)

        # Subscribers
        rospy.Subscriber("/cf1/laser_reading_tot", Float64MultiArray, self.clbk_laser)
        rospy.Subscriber('/cf1/odom_absolute',Odometry,self.clbk_odom)

        # Wait for the service
        rospy.wait_for_service('/go_to_point_switch')
        rospy.wait_for_service('/wall_follower_switch')

        # Service clients
        self.srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
        self.srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)

        # Initialize the state to go_to_point
        self.change_state(0)

    def clbk_laser(self, msg):
        # Get laser reading from subscribed topic
        self.front = msg.data[0]
        self.back = msg.data[1]
        self.left = msg.data[2]
        self.right = msg.data[3]
        self.top = msg.data[4]
        self.bottom = msg.data[5]

    def clbk_odom(self,msg):
        # Get ground truth position
        self.odom_position_ = msg.pose.pose.position

        # Get ground yaw
        self.quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        self.euler_angle = transformations.euler_from_quaternion(self.quaternion)
        self.odom_yaw_ = self.euler_angle[2]
    
    def change_state(self, state):
        self.state_ = state
        rospy.loginfo("state changed: %s" % self.state_disc_[state])
        if self.state_ == 0:
            self.resp = self.srv_client_go_to_point_(True)
            self.resp = self.srv_client_wall_follower_(False)
        if self.state_ == 1:
            self.resp = self.srv_client_go_to_point_(False)
            self.resp = self.srv_client_wall_follower_(True)

    def normalize_angle(self, angle):
        if(math.fabs(angle) > math.pi):
            self.angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return self.angle

    # Decide different state and corresponding action
    def take_action_bug0(self):
        while not rospy.is_shutdown():
            
            # if self.front==0 and self.back==0 and self.left==0 and self.right==0 and self.top==0 and self.bottom==0:
            #     continue
            
            if self.state_ == 0:
                if self.front > 0.15 and self.front < 1:
                    self.change_state(1)
            
            elif self.state_ == 1:
                self.desired_yaw = math.atan2(self.desired_position_.y - self.odom_position_.y, self.desired_position_.x - self.odom_position_.x)
                self.err_yaw = self.normalize_angle(self.desired_yaw - self.odom_yaw_)
                
                 # less than 45 degrees
                if math.fabs(self.err_yaw) < (math.pi / 4) and \
                   self.front > 1.5 and self.right > 1:
                    rospy.loginfo('less than 45')
                    self.change_state(0)

                # between 45 and 90
                if self.err_yaw > 0 and \
                   math.fabs(self.err_yaw) > (math.pi / 4) and \
                   math.fabs(self.err_yaw) < (math.pi / 2) and \
                   self.left > 1.5:
                    rospy.loginfo('between 45 and 90 - to the left')
                    self.change_state(0)

                if self.err_yaw < 0 and \
                   math.fabs(self.err_yaw) > (math.pi / 4) and \
                   math.fabs(self.err_yaw) < (math.pi / 2) and \
                   self.right > 1.5:
                    rospy.loginfo('between 45 and 90 - to the right')
                    self.change_state(0)                       
                                
                
                # if math.fabs(self.err_yaw) < (math.pi / 6) and \
                #    self.front > 1.5:
                #    self.change_state(0)
            
                # if self.err_yaw > 0 and \
                #    math.fabs(self.err_yaw) > (math.pi / 4) and \
                #    math.fabs(self.err_yaw) < (math.pi / 2) and \
                #    self.left > 1.5:
                #    self.change_state(0)
                
                # if self.err_yaw < 0 and \
                #    math.fabs(self.err_yaw) > (math.pi / 4) and \
                #    math.fabs(self.err_yaw) < (math.pi / 2) and \
                #    self.right > 1.5:
                #    self.change_state(0)



                #  # less than 30 degrees
                # if math.fabs(self.err_yaw) < (math.pi / 6) and \
                #    self.front > 1.5 and self.right > 1 and self.left > 1:
                #     rospy.loginfo('less than 30')
                #     self.change_state(0)

                # # between 30 and 90
                # if self.err_yaw > 0 and \
                #    math.fabs(self.err_yaw) > (math.pi / 6) and \
                #    math.fabs(self.err_yaw) < (math.pi / 2) and \
                #    self.left > 1:
                #     rospy.loginfo('between 30 and 90 - to the left')
                #     self.change_state(0)

                # if self.err_yaw < 0 and \
                #    math.fabs(self.err_yaw) > (math.pi / 6) and \
                #    math.fabs(self.err_yaw) < (math.pi / 2) and \
                #    self.right > 1:
                #     rospy.loginfo('between 30 and 90 - to the right')
                #     self.change_state(0)


                # if self.err_yaw > 0 and \
                #    math.fabs(self.err_yaw) > (math.pi / 4) and \
                #    self.front > 1.5:
                #     self.change_state(0)

                # if self.err_yaw > 0 and \
                #    math.fabs(self.err_yaw) > (math.pi / 4) and \
                #    math.fabs(self.err_yaw) < (math.pi / 2) and \
                #    self.left > 1.5:
                #     self.change_state(0)
                
                # if self.err_yaw < 0 and \
                #    math.fabs(self.err_yaw) > (math.pi / 4) and \
                #    math.fabs(self.err_yaw) < (math.pi / 2) and \
                #    self.right > 1.5:
                #     self.change_state(0)

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node("bug0")
    my_node = Bug0()
    my_node.take_action_bug0()
    



