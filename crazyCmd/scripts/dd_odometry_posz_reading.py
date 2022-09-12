#! /usr/bin/env python3

import rospy

from nav_msgs.msg import Odometry

def clbk_odometry(msg):
    rospy.loginfo(msg.pose.pose.position.z)

def main():
    rospy.init_node("odometry_posz_reading")
    sub = rospy.Subscriber('/cf1/odom_absolute',Odometry,clbk_odometry)
    rospy.spin()

if __name__ == '__main__':
    main()