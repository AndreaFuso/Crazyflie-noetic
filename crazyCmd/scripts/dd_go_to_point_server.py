#! /usr/bin/env python3

# The difference between go_to_point.py is that
# 1) we add a service server, which will be used in Bug) algorithm.

import rospy

from crazyflie_messages.msg import Position
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math


class GoToPoint():
    def __init__(self):
        # Initialize odometry components
        self.odom_position_ = Point()
        self.odom_yaw_ = 0

        # Other parameters 
        self.quaternion = None
        self.euler_angle = None
        self.cmd_msg = Position()
        self.angle = 0
        
        # The flag for Bug0
        self.active_ = False

        # Machine state
        self.machine_state_ = 0
        self.machine_state_dict_ = {
            0: 'fix_yaw',
            1: 'go_straight_ahead',
            2: 'done',
        }

        # Desired goal point
        self.desired_position_ = Point()
        self.desired_position_.x = rospy.get_param('des_pos_x')
        self.desired_position_.y = rospy.get_param('des_pos_y')
        # self.desired_position_.x = -3
        # self.desired_position_.y = 7
        self.desired_position_.z = 0.5

        # Precision parameters
        self.yaw_precision_ = 2*(math.pi/180) # +/- 5 degrees allowed
        self.dist_precision_ = 0.3

        #Node cycle rate
        self.rate = rospy.Rate(100)

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cf1/cmd_vel', Position, queue_size=1)

        # Subscribers
        rospy.Subscriber('/cf1/odom_absolute',Odometry,self.clbk_odom)

        # Service servers
        rospy.Service('go_to_point_switch', SetBool, self.go_to_point_switch)
        rospy.loginfo("Go_to_point server is ready")

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
    
    def go_to_point_switch(self, req):
        self.active_ = req.data
        self.res = SetBoolResponse()
        self.res.success = True
        self.res.message = 'Done!'
        return self.res

    def change_state(self, state):
        self.machine_state_ = state
        rospy.loginfo('Go to point - [%s] - %s' % (state, self.machine_state_dict_[state]))
    
    def fix_yaw(self, des_pos):
        self.desired_yaw = math.atan2(des_pos.y - self.odom_position_.y, des_pos.x - self.odom_position_.x )      
        self.err_yaw = self.desired_yaw - self.odom_yaw_
        
        self.cmd_msg = Position()
        if math.fabs(self.err_yaw) > self.yaw_precision_:
            self.cmd_msg.desired_yaw_rate = 5 if self.err_yaw > 0 else -5
            self.cmd_msg.desired_velocity.x = 0
        self.cmd_vel_pub.publish(self.cmd_msg)

        # State change conditions
        if math.fabs(self.err_yaw) <= self.yaw_precision_:
            # rospy.loginfo('Yaw error: [%s]' % self.err_yaw)
            self.change_state(1)

    def go_straight_ahead(self,des_pos):
        self.desired_yaw = math.atan2(des_pos.y - self.odom_position_.y, des_pos.x - self.odom_position_.x )      
        self.err_yaw = self.desired_yaw - self.odom_yaw_
        self.err_pos = math.sqrt(pow(des_pos.y - self.odom_position_.y,2)+pow(des_pos.x - self.odom_position_.x,2))

        if self.err_pos > self.dist_precision_:
            self.cmd_msg.desired_velocity.x = 0.3
            self.cmd_msg.desired_yaw_rate = 0
            self.cmd_vel_pub.publish(self.cmd_msg)
        else:
            rospy.loginfo('Position error: [%s]' % self.err_pos)
            self.change_state(2)

        # state change conditions
        if math.fabs(self.err_yaw) > self.yaw_precision_:
            rospy.loginfo('Yaw error: [%s]' % self.err_yaw)
            self.change_state(0)

    def done(self):
        self.cmd_msg.desired_velocity.x = 0
        self.cmd_msg.desired_yaw_rate = 0
        self.cmd_vel_pub.publish(self.cmd_msg)

    # Decide different state and corresponding action
    def take_action_go_to_point(self):
        while not rospy.is_shutdown():
            if not self.active_: 
                continue
            else:
                if self.machine_state_ == 0:
                    self.fix_yaw(self.desired_position_)
                elif self.machine_state_ == 1:
                    self.go_straight_ahead(self.desired_position_)
                elif self.machine_state_ == 2:
                    self.done()
                    pass
                else:
                    rospy.logerr('Unknown state!')
                    pass
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node("go_to_point_server")
    my_node = GoToPoint()
    my_node.take_action_go_to_point()
    



