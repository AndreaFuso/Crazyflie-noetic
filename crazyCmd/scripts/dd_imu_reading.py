#! /usr/bin/env python3

import rospy
import tf

from sensor_msgs.msg import Imu
from crazyflie_messages.msg import RollPitchYaw

def clbk_imu(msg):
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z,
             msg.orientation.w])

    rpy_msg = RollPitchYaw()
    rpy_msg.roll = roll
    rpy_msg.pitch = pitch
    rpy_msg.yaw = yaw
    rpy_pub.publish(rpy_msg)

def main():
    global rpy_pub
    rospy.init_node("imu_reading")
    sub = rospy.Subscriber('/cf1/imu',Imu,clbk_imu)
    rpy_pub = rospy.Publisher('/cf1/rpy_imu', RollPitchYaw, queue_size=1)
    

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()