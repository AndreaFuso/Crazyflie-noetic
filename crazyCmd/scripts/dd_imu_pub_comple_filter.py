#! /usr/bin/env python3

import rospy
import tf

from sensor_msgs.msg import Imu

def clbk_imu(msg):

    imu_pub.publish(msg)

def main():
    global imu_pub
    rospy.init_node("imu_reading")
    sub = rospy.Subscriber('/cf1/imu',Imu,clbk_imu)
    imu_pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=1)  

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()