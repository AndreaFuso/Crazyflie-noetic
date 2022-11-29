#! /usr/bin/env python3

# This is the server to do the go straight operation, used in dd_room_mapping_bug1.py
# Just simply implement a linear x velocity

import rospy

from crazyflie_messages.msg import Position
from std_srvs.srv import *


class GoStraight():
    def __init__(self):

        self.cmd_msg = Position()
        
        # The flag for Bug0
        self.active_ = False

        # Machine state
        self.machine_state_ = 0

        #Node cycle rate
        self.rate = rospy.Rate(100)

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cf1/cmd_vel', Position, queue_size=1)

        # Service servers
        rospy.Service('go_straight_switch', SetBool, self.go_straight_switch)
        rospy.loginfo("Go_straight server is ready")
    
    def go_straight_switch(self, req):
        self.active_ = req.data
        self.res = SetBoolResponse()
        self.res.success = True
        self.res.message = 'Done!'
        return self.res

    def go_straight_ahead(self):

        self.cmd_msg.desired_velocity.x = 0.1
        self.cmd_msg.desired_yaw_rate = 0
        self.cmd_vel_pub.publish(self.cmd_msg)

    def take_action_go_straight(self):
        while not rospy.is_shutdown():
            if not self.active_: 
                continue
            else:
                if self.machine_state_ == 0:
                    self.go_straight_ahead()
                else:
                    rospy.logerr('Unknown state!')
                    pass
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node("go_straight_server")
    my_node = GoStraight()
    my_node.take_action_go_straight()
    



