#! /usr/bin/env python3

# here we do the obstacle avoidance, before it, we need to run laser_reading_collection.py
import rospy

from std_msgs.msg import Float64MultiArray
from crazyflie_messages.msg import Position

class ObstacleAvoidance():
    def __init__(self):
        # Initialize parameters
        self.velocity = 0
        self.angular_z = 0
        self.limit = 1
        self.state_description = ""
        self.cmd = Position()

        #Node cycle rate
        self.rate = rospy.Rate(100)

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cf1/cmd_vel', Position, queue_size=1)

        # Subscribers
        rospy.Subscriber("/cf1/laser_reading_tot", Float64MultiArray, self.clbk)

    def clbk(self, msg):
        # Get laser reading from subscribed topic
        self.front = msg.data[0]
        self.back = msg.data[1]
        self.left = msg.data[2]
        self.right = msg.data[3]
        self.top = msg.data[4]
        self.bottom = msg.data[5]

        # Implement the obstacle avoidance logic
        if self.front > self.limit and self.left > self.limit and self.right > self.limit and self.back > self.limit:
            self.state_description = "case1: nothing"
            self.velocity = 0.3
            self.angular_z = 0
        elif self.front < self.limit and self.left > self.limit and self.right > self.limit and self.back > self.limit:
            self.state_description = "case2: obstacle in the front"
            self.velocity = 0
            self.angular_z = 5
        elif self.front > self.limit and self.left < self.limit and self.right > self.limit and self.back > self.limit:
            self.state_description = "case3: obstacle in the left"
            self.velocity = 0.3
            self.angular_z = 0  
        elif self.front > self.limit and self.left > self.limit and self.right < self.limit and self.back > self.limit:
            self.state_description = "case4: obstacle in the right"
            self.velocity = 0.3
            self.angular_z = 0
        elif self.front > self.limit and self.left > self.limit and self.right > self.limit and self.back < self.limit:
            self.state_description = "case5: obstacle in the back"
            self.velocity = 0.3
            self.angular_z = 0
        elif self.front < self.limit and self.left < self.limit and self.right > self.limit and self.back > self.limit:
            self.state_description = "case6: obstacle in the front and left"
            self.velocity = 0
            self.angular_z = -5
        elif self.front < self.limit and self.left > self.limit and self.right < self.limit and self.back > self.limit:
            self.state_description = "case7: obstacle in the front and right"
            self.velocity = 0
            self.angular_z = 5
        elif self.front < self.limit and self.left > self.limit and self.right > self.limit and self.back < self.limit:
            self.state_description = "case8: obstacle in the front and back "
            self.velocity = 0
            self.angular_z = 5
        elif self.front > self.limit and self.left < self.limit and self.right < self.limit and self.back > self.limit:
            self.state_description = "case9: obstacle in the left and right "
            self.velocity = 0.3
            self.angular_z = 0
        elif self.front > self.limit and self.left < self.limit and self.right > self.limit and self.back < self.limit:
            self.state_description = "case10: obstacle in the left and back "
            self.velocity = 0.3
            self.angular_z = 0
        elif self.front > self.limit and self.left > self.limit and self.right < self.limit and self.back < self.limit:
            self.state_description = "case11: obstacle in the right and back "
            self.velocity = 0.3
            self.angular_z = 0
        elif self.front < self.limit and self.left < self.limit and self.right < self.limit and self.back > self.limit:
            self.state_description = "case12: obstacle in the front, left and right "
            self.velocity = 0
            self.angular_z = 5
        elif self.front < self.limit and self.left < self.limit and self.right > self.limit and self.back < self.limit:
            self.state_description = "case13: obstacle in the front, left and back "
            self.velocity = 0
            self.angular_z = -5
        elif self.front < self.limit and self.left > self.limit and self.right < self.limit and self.back < self.limit:
            self.state_description = "case14: obstacle in the front, right and back "
            self.velocity = 0
            self.angular_z = 5
        elif self.front > self.limit and self.left < self.limit and self.right < self.limit and self.back < self.limit:
            self.state_description = "case15: obstacle in the left, right and back"
            self.velocity = 0.3
            self.angular_z = 0
        else:
            self.state_description = "unknown case"
        
        # Display the laser reading message and current state
        rospy.loginfo("front:{}, back:{}, left:{}, right:{}, top:{}, bottom:{}".format(self.front, self.back, self.left, self.right, self.top, self.bottom))
        rospy.loginfo(self.state_description)

    # Publish the desired velocity components
    def take_action_obstacle_avoidance(self):
        while not rospy.is_shutdown():
            self.cmd.desired_velocity.x = self.velocity
            self.cmd.desired_yaw_rate = self.angular_z
            self.cmd_vel_pub.publish(self.cmd)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node("obstacle_avoidance")
    my_node = ObstacleAvoidance()
    my_node.take_action_obstacle_avoidance()
    



