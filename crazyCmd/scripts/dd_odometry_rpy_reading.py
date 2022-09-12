#! /usr/bin/env python3

import rospy
import tf

from nav_msgs.msg import Odometry
from crazyflie_messages.msg import RollPitchYaw

def clbk_odometry(msg):

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w])
    
    rpy_msg = RollPitchYaw()
    rpy_msg.roll = roll
    rpy_msg.pitch = pitch
    rpy_msg.yaw = yaw
    rpy_pub.publish(rpy_msg)

    # rospy.loginfo([roll,pitch,yaw])

def main():
    global rpy_pub
    rospy.init_node("odometry_rpy_reading")
    sub = rospy.Subscriber('/cf1/odom_absolute',Odometry,clbk_odometry)
    rpy_pub = rospy.Publisher('/cf1/rpy_odom', RollPitchYaw, queue_size=1)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()