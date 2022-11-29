#! /usr/bin/env python3

# here we do the follow the wall , before it, we need to run laser_reading_collection.py
# 20220921 remark: up to now, not very good because the big fluctuation when we change between turn_left and follow_the_wall
# 20220921 remark: now we set the velocity to very small values, in this way we can have a small fluctuation

# The difference between go_to_point.py is that
# 1) we add a service server, which will be used in Bug) algorithm.

import rospy

from std_msgs.msg import Float64MultiArray
from crazyflie_messages.msg import Position
from std_srvs.srv import *


class FollowWall():
    def __init__(self):
        # Initialize odometry components
        self.front, self.back, self.left, self.right, self.top, self.bottom = 0,0,0,0,0,0
        self.state_description = ""
        self.limit = 1.5
        self.cmd_msg = Position()

        # The flag for Bug0
        self.active_ = False
        
        # Machine state
        self.state_ = 0
        self.state_dict_ = {
            0: 'find the wall',
            1: 'turn left',
            2: 'turn left with velocity',
            3:'follow the wall',
        }

        #Node cycle rate
        self.rate = rospy.Rate(100)

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cf1/cmd_vel', Position, queue_size=1)

        # Subscribers
        rospy.Subscriber("/cf1/laser_reading_tot", Float64MultiArray, self.clbk_laser)

        # Service servers
        rospy.Service('wall_follower_switch', SetBool, self.wall_follower_switch)
        rospy.loginfo("Wall_follower server is ready")

    def clbk_laser(self, msg):
        # Get laser reading from subscribed topic
        self.front = msg.data[0]
        self.back = msg.data[1]
        self.left = msg.data[2]
        self.right = msg.data[3]
        self.top = msg.data[4]
        self.bottom = msg.data[5]

        self.decide_state()

    def wall_follower_switch(self, req):
        self.active_ = req.data
        self.res = SetBoolResponse()
        self.res.success = True
        self.res.message = 'Done!'
        return self.res
    
    def change_state(self,state):
        if state is not self.state_:
            rospy.loginfo('Wall follower - [%s] - %s' % (state, self.state_dict_[state]))
            self.state_ = state
            
### ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  ###

    def decide_state(self):
        if self.front > self.limit and self.left > self.limit and self.right > self.limit and self.back > self.limit:
            self.state_description = "case1: nothing"
            self.change_state(0)
        elif self.front < self.limit and self.left > self.limit and self.right > self.limit and self.back > self.limit:
            self.state_description = "case2: obstacle in the front"
            self.change_state(1)
        elif self.front > self.limit and self.left < self.limit and self.right > self.limit and self.back > self.limit:
            self.state_description = "case3: obstacle in the left"
            self.change_state(0)  
        elif self.front > self.limit and self.left > self.limit and self.right < self.limit and self.back > self.limit:
            self.state_description = "case4: obstacle in the right"
            self.change_state(3)
        elif self.front > self.limit and self.left > self.limit and self.right > self.limit and self.back < self.limit:
            self.state_description = "case5: obstacle in the back"
            self.change_state(0)
        elif self.front < self.limit and self.left < self.limit and self.right > self.limit and self.back > self.limit:
            self.state_description = "case6: obstacle in the front and left"
            self.change_state(1)
        elif self.front < self.limit and self.left > self.limit and self.right < self.limit and self.back > self.limit:
            self.state_description = "case7: obstacle in the front and right"
            self.change_state(1)
        elif self.front < self.limit and self.left > self.limit and self.right > self.limit and self.back < self.limit:
            self.state_description = "case8: obstacle in the front and back "
            self.change_state(1)
        elif self.front > self.limit and self.left < self.limit and self.right < self.limit and self.back > self.limit:
            self.state_description = "case9: obstacle in the left and right "
            self.change_state(0)
        elif self.front > self.limit and self.left < self.limit and self.right > self.limit and self.back < self.limit:
            self.state_description = "case10: obstacle in the left and back "
            self.change_state(0)
        elif self.front > self.limit and self.left > self.limit and self.right < self.limit and self.back < self.limit:
            self.state_description = "case11: obstacle in the right and back "
            self.change_state(3)
        elif self.front < self.limit and self.left < self.limit and self.right < self.limit and self.back > self.limit:
            self.state_description = "case12: obstacle in the front, left and right "
            self.change_state(1)
        elif self.front < self.limit and self.left < self.limit and self.right > self.limit and self.back < self.limit:
            self.state_description = "case13: obstacle in the front, left and back "
            self.change_state(1)
        elif self.front < self.limit and self.left > self.limit and self.right < self.limit and self.back < self.limit:
            self.state_description = "case14: obstacle in the front, right and back "
            self.change_state(1)
        elif self.front > self.limit and self.left < self.limit and self.right < self.limit and self.back < self.limit:
            self.state_description = "case15: obstacle in the left, right and back"
            self.change_state(0)
        else:
            self.state_description = "unknown case"
            rospy.loginfo("front:{}, back:{}, left:{}, right:{}, top:{}, bottom:{}".format(self.front, self.back, self.left, self.right, self.top, self.bottom))

    def find_wall(self):
        self.cmd_msg.desired_velocity.x = 0.2
        self.cmd_msg.desired_yaw_rate = -5
        return self.cmd_msg
    
    def turn_left(self):
        self.cmd_msg.desired_velocity.x = 0
        self.cmd_msg.desired_yaw_rate = 15
        return self.cmd_msg

    # def turn_right(self):
    #     self.cmd_msg.desired_velocity.x = 0
    #     self.cmd_msg.desired_yaw_rate = -10
    #     return self.cmd_msg

    def follow_the_wall(self):
        self.cmd_msg.desired_velocity.x = 0.1
        self.cmd_msg.desired_yaw_rate = 0
        return self.cmd_msg

### ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  ###


### ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  ###

    # def decide_state(self):
    #     if self.right < 0.5:
    #         self.state_description = "case1: too close to right"
    #         self.change_state(2)
    #     else:
    #         if self.front > self.limit and self.left > self.limit and self.right > self.limit and self.back > self.limit:
    #             self.state_description = "case1: nothing"
    #             self.change_state(0)
    #         elif self.front < self.limit and self.left > self.limit and self.right > self.limit and self.back > self.limit:
    #             self.state_description = "case2: obstacle in the front"
    #             self.change_state(1)
    #         elif self.front > self.limit and self.left < self.limit and self.right > self.limit and self.back > self.limit:
    #             self.state_description = "case3: obstacle in the left"
    #             self.change_state(0)  
    #         elif self.front > self.limit and self.left > self.limit and self.right < self.limit and self.back > self.limit:
    #             self.state_description = "case4: obstacle in the right"
    #             self.change_state(3)
    #         elif self.front > self.limit and self.left > self.limit and self.right > self.limit and self.back < self.limit:
    #             self.state_description = "case5: obstacle in the back"
    #             self.change_state(0)
    #         elif self.front < self.limit and self.left < self.limit and self.right > self.limit and self.back > self.limit:
    #             self.state_description = "case6: obstacle in the front and left"
    #             self.change_state(1)
    #         elif self.front < self.limit and self.left > self.limit and self.right < self.limit and self.back > self.limit:
    #             self.state_description = "case7: obstacle in the front and right"
    #             self.change_state(1)
    #         elif self.front < self.limit and self.left > self.limit and self.right > self.limit and self.back < self.limit:
    #             self.state_description = "case8: obstacle in the front and back "
    #             self.change_state(1)
    #         elif self.front > self.limit and self.left < self.limit and self.right < self.limit and self.back > self.limit:
    #             self.state_description = "case9: obstacle in the left and right "
    #             self.change_state(0)
    #         elif self.front > self.limit and self.left < self.limit and self.right > self.limit and self.back < self.limit:
    #             self.state_description = "case10: obstacle in the left and back "
    #             self.change_state(0)
    #         elif self.front > self.limit and self.left > self.limit and self.right < self.limit and self.back < self.limit:
    #             self.state_description = "case11: obstacle in the right and back "
    #             self.change_state(3)
    #         elif self.front < self.limit and self.left < self.limit and self.right < self.limit and self.back > self.limit:
    #             self.state_description = "case12: obstacle in the front, left and right "
    #             self.change_state(1)
    #         elif self.front < self.limit and self.left < self.limit and self.right > self.limit and self.back < self.limit:
    #             self.state_description = "case13: obstacle in the front, left and back "
    #             self.change_state(1)
    #         elif self.front < self.limit and self.left > self.limit and self.right < self.limit and self.back < self.limit:
    #             self.state_description = "case14: obstacle in the front, right and back "
    #             self.change_state(1)
    #         elif self.front > self.limit and self.left < self.limit and self.right < self.limit and self.back < self.limit:
    #             self.state_description = "case15: obstacle in the left, right and back"
    #             self.change_state(0)
    #         else:
    #             self.state_description = "unknown case"
    #             rospy.loginfo("front:{}, back:{}, left:{}, right:{}, top:{}, bottom:{}".format(self.front, self.back, self.left, self.right, self.top, self.bottom))

    # def find_wall(self):
    #     self.cmd_msg.desired_velocity.x = 0.2
    #     self.cmd_msg.desired_yaw_rate = -15
    #     return self.cmd_msg
    
    # def turn_left(self):
    #     self.cmd_msg.desired_velocity.x = 0
    #     self.cmd_msg.desired_yaw_rate = 15
    #     return self.cmd_msg

    # def turn_left_with_velocity(self):
    #     self.cmd_msg.desired_velocity.x = 0.2
    #     self.cmd_msg.desired_yaw_rate = 4
    #     return self.cmd_msg

    # def follow_the_wall(self):
    #     self.cmd_msg.desired_velocity.x = 0.2
    #     self.cmd_msg.desired_yaw_rate = 0
    #     return self.cmd_msg

### ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  ###


    # Decide different state and corresponding action
    def take_action_follow_wall(self):
        while not rospy.is_shutdown():
            if not self.active_: 
                continue
            else:
                if self.state_ == 2:
                    self.cmd_msg = self.turn_left_with_velocity()
                else:
                    if self.state_ == 0:
                        self.cmd_msg = self.find_wall()
                    elif self.state_ == 1:
                        self.cmd_msg = self.turn_left()                        
                    elif self.state_ == 3:
                        self.cmd_msg = self.follow_the_wall()
                        pass
                    else:
                        rospy.logerr('Unknown state!')
            self.cmd_vel_pub.publish(self.cmd_msg)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node("follow_wall_server")
    my_node = FollowWall()
    my_node.take_action_follow_wall()
    



