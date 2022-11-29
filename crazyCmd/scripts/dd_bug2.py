#! /usr/bin/env python3

# Here is the Bug2 algorithm, 
# One more function distance_to_line(). It calculates the distance of the robot from the imaginary line 
# that joins initial position of the robot with the desired position of the robot.

# Bug 2 algorithm is to follow this imaginary line in absense of obstacle (remember go to point). 
# When an obstacle shows up, the robot starts circumnavigating it till it again finds itself close to the imaginary line.



import rospy

from std_msgs.msg import Float64MultiArray
from crazyflie_messages.msg import Position
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_srvs.srv import *
from tf import transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

import math


class Bug2():
    def __init__(self):
        # Initialize parameters
        self.front, self.back, self.left, self.right, self.top, self.bottom = 0,0,0,0,0,0
        self.odom_yaw_ = 0
        self.yaw_error_allowed_ = 5*(math.pi/180) # +/- 5 degrees
        self.odom_position_ = Point()
        self.angle = 0

        self.initial_position_ = Point()
        # self.initial_position_.x = rospy.get_param('initial_x')
        # self.initial_position_.y = rospy.get_param('initial_y')
        self.initial_position_.x = 0
        self.initial_position_.y = 7
        self.initial_position_.z = 0.5
        self.p1 = self.initial_position_

        self.desired_position_ = Point()
        self.desired_position_.x = rospy.get_param('des_pos_x')
        self.desired_position_.y = rospy.get_param('des_pos_y')
        self.desired_position_.z = 0.5
        self.p2 = self.desired_position_

        self.desired_yaw = 0
        self.err_yaw = 0

        # Initialize the line equation
        self.up_eq = None
        self.lo_eq = None
        self.distance = 0

        # Initialize service clients
        self.srv_client_go_to_point_ = None
        self.srv_client_wall_follower_ = None
        
        # Machine state
        self.state_ = 0 
        # 0 - go to point
        # 1 - wall following

        self.state_disc_ = ['Go to point', 'Wall following']

        # Count state time and count loop
        self.count_state_time = 0 # seconds the robot is in a state
        self.count_loop_ = 0      

        #Node cycle rate
        self.rate = rospy.Rate(20)

        # Subscribers
        rospy.Subscriber("/cf1/laser_reading_tot", Float64MultiArray, self.clbk_laser)
        rospy.Subscriber('/cf1/odom_absolute',Odometry,self.clbk_odom)

        # Wait for the service
        rospy.wait_for_service('/go_to_point_switch')
        rospy.wait_for_service('/wall_follower_switch')
        # rospy.wait_for_service('/gazebo/set_model_state')

        # Service clients
        self.srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
        self.srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)
        # srv_client_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # set robot position
        self.model_state = ModelState()
        self.model_state.model_name = 'cf1'
        self.model_state.pose.position.x = self.initial_position_.x
        self.model_state.pose.position.y = self.initial_position_.y
        # self.resp = srv_client_set_model_state(self.model_state)

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
        self.count_state_time = 0
        self.state_ = state
        rospy.loginfo("state changed: %s" % self.state_disc_[state])
        if self.state_ == 0:
            self.resp = self.srv_client_go_to_point_(True)
            self.resp = self.srv_client_wall_follower_(False)
        if self.state_ == 1:
            self.resp = self.srv_client_go_to_point_(False)
            self.resp = self.srv_client_wall_follower_(True)
    
    def distance_to_line(self, p0):
        # p0 is the current position
        # p1 and p2 points define the line: p1 is initial position, p2 is desired position
        # here goes the line equation

        self.up_eq = math.fabs((self.p2.y - self.p1.y) * p0.x - (self.p2.x - self.p1.x) * p0.y + (self.p2.x * self.p1.y) - (self.p2.y * self.p1.x))
        self.lo_eq = math.sqrt(pow(self.p2.y - self.p1.y, 2) + pow(self.p2.x - self.p1.x, 2))
        self.distance = self.up_eq / self.lo_eq
        return self.distance    

    # def normalize_angle(self, angle):
    #     if(math.fabs(angle) > math.pi):
    #         self.angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    #     return self.angle

    # Decide different state and corresponding action
    def take_action_bug2(self):
        while not rospy.is_shutdown():
            
            if self.front==0 and self.back==0 and self.left==0 and self.right==0 and self.top==0 and self.bottom==0:
                continue
            
            self.distance_position_to_line = self.distance_to_line(self.odom_position_)

            if self.state_ == 0:
                if self.front > 0.15 and self.front < 1.5:
                    self.change_state(1)
            
            elif self.state_ == 1:
                if self.count_state_time > 5 and self.distance_position_to_line < 0.5:
                    self.change_state(0)
                    # rospy.loginfo("distance to line: [%.2f], position: [%.2f, %.2f]", self.distance_to_line(self.odom_position_), self.odom_position_.x, self.odom_position_.y)
            
            self.count_loop_ = self.count_loop_ + 1
            if self.count_loop_ == 20:
                self.count_state_time = self.count_state_time + 1
                self.count_loop_ = 0
            
            # rospy.loginfo(self.count_state_time)            
            # rospy.loginfo("distance to line: [%.2f], position: [%.2f, %.2f]", self.distance_to_line(self.odom_position_), self.odom_position_.x, self.odom_position_.y)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node("bug2")
    my_node = Bug2()
    my_node.take_action_bug2()
    



