#! /usr/bin/env python3

# Here is the Bug1 algorithm which moves the robot about the obstacle (circumnavigate). 
# When the robot passes near the goal it records this point and keeps on circumnavigating the obstacle. 
# Once the robot reaches the initial point (where the robot first met the obstacle) it then goes to the point stored in memory and then moves towards the goal from there.

# We will have three main states:1) Go to point  2) Circumnavigate the obstacle  3) Go to closest point
# Use world03, and run laser_reading_collection before running bug1.launch

import rospy

from std_msgs.msg import Float64MultiArray
from crazyflie_messages.msg import Position
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_srvs.srv import *
from tf import transformations

import math


class Bug1():
    def __init__(self):
        # Initialize parameters
        self.front, self.back, self.left, self.right, self.top, self.bottom = 0,0,0,0,0,0
        self.odom_yaw_ = 0
        self.yaw_error_allowed_ = 5*(math.pi/180) # +/- 5 degrees
        self.odom_position_ = Point() # also for the circumanavigate start and closest point
        
        self.angle = 0

        self.desired_position_ = Point()
        self.desired_position_.x = rospy.get_param('des_pos_x')
        self.desired_position_.y = rospy.get_param('des_pos_y')
        self.desired_position_.z = 0.5

        self.desired_yaw = 0
        self.err_yaw = 0

        # Circumnavigating parameters
        self.circumnavigate_starting_point_ = Point()
        self.circumnavigate_closest_point_ = Point()
        
        # Used for counting the time in a state
        self.count_state_time_ = 0 # seconds the robot is in a state
        self.count_loop_ = 0

        # Initialize service clients
        self.srv_client_go_to_point_ = None
        self.srv_client_wall_follower_ = None
        
        # Machine state
        self.state_ = 0 
        # 0 - go to point
        # 1 - circumnavigate obstacle
        # 2 - go to closest point

        self.state_disc_ = ['Go to point', 'Circumnavigate obstacle', 'Go to closest point']
        
        #Node cycle rate
        self.rate = rospy.Rate(20)

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
        self.count_state_time_ = 0
        self.state_ = state
        rospy.loginfo("state changed: %s" % self.state_disc_[state])
        if self.state_ == 0:
            self.resp = self.srv_client_go_to_point_(True)
            self.resp = self.srv_client_wall_follower_(False)
        if self.state_ == 1:
            self.resp = self.srv_client_go_to_point_(False)
            self.resp = self.srv_client_wall_follower_(True)
        if self.state_ == 2:
            self.resp = self.srv_client_go_to_point_(False)
            self.resp = self.srv_client_wall_follower_(True)

    def calc_dist_points(self, point1, point2):
        self.dist = math.sqrt((point1.y - point2.y)**2 + (point1.x - point2.x)**2)
        return self.dist

    def normalize_angle(self, angle):
        if(math.fabs(angle) > math.pi):
            self.angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return self.angle

    # Decide different state and corresponding action
    def take_action_bug1(self):
        while not rospy.is_shutdown():
            
            if self.front==0 and self.back==0 and self.left==0 and self.right==0 and self.top==0 and self.bottom==0:
                continue
            
            if self.state_ == 0:
                if self.front > 0.15 and self.front < 1:
                    self.circumnavigate_closest_point_ = self.odom_position_
                    self.circumnavigate_starting_point_ = self.odom_position_
                    self.change_state(1)
            
            elif self.state_ == 1:
                # if current position is closer to the goal than the previous closest_position, 
                # assign current position to closest_point
                if self.calc_dist_points(self.odom_position_, self.desired_position_) < self.calc_dist_points(self.circumnavigate_closest_point_, self.desired_position_):
                    self.circumnavigate_closest_point_ = self.odom_position_

                # compare only after 5 seconds - need some time to get out of starting_point
                # if robot reaches (is close to) starting point     
                if self.count_state_time_ > 20 and \
                   self.calc_dist_points(self.odom_position_, self.circumnavigate_starting_point_) < 1:
                    self.change_state(2)

            elif self.state_ == 2:
                 # if robot reaches (is close to) closest point
                 if self.calc_dist_points(self.odom_position_, self.circumnavigate_closest_point_) < 2:
                    self.change_state(0)   

            self.count_loop_ = self.count_loop_ + 1
            if self.count_loop_ == 20:
                self.count_state_time_ = self.count_state_time_ + 1
                self.count_loop_ = 0
            # rospy.loginfo("{}".format(self.circumnavigate_closest_point_))        
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node("bug1")
    my_node = Bug1()
    my_node.take_action_bug1()
    


