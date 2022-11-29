#! /usr/bin/env python3
# here we just publish the data of topic '/cf1/imu' to another topic '/imu/data_raw';
# in this way, we can use the package 'imu_complementary_filter', because '/imu/data_raw' is the defined topic for that package

import rospy
import tf

from sensor_msgs.msg import Imu

def clbk_imu(msg):
    
    imu_pub_comple_filter.publish(msg)
    # msg_ekf = Imu()
    # msg_ekf.header.seq = msg.header.seq
    # msg_ekf.header.stamp.secs = msg.header.stamp.secs
    # msg_ekf.header.stamp.nsecs = msg.header.stamp.nsecs
    # msg_ekf.header.frame_id = "crazyflie_main_body"
    # msg_ekf.orientation.x = msg.orientation.x
    # msg_ekf.orientation.y = msg.orientation.y
    # msg_ekf.orientation.z = msg.orientation.z
    # msg_ekf.orientation_covariance = msg.orientation_covariance
    # msg_ekf.angular_velocity.x = msg.angular_velocity.x
    # msg_ekf.angular_velocity.y = msg.angular_velocity.y
    # msg_ekf.angular_velocity.z = msg.angular_velocity.z
    # msg_ekf.angular_velocity_covariance = msg.angular_velocity_covariance
    # msg_ekf.linear_acceleration.x = msg.linear_acceleration.x
    # msg_ekf.linear_acceleration.y = msg.linear_acceleration.y
    # msg_ekf.linear_acceleration.z = msg.linear_acceleration.z
    # msg_ekf.linear_acceleration_covariance = msg.linear_acceleration_covariance


    # imu_pub_ekf.publish(msg)

def main():
    global imu_pub_comple_filter
    # , imu_pub_ekf
    rospy.init_node("imu_topic_change")
    sub = rospy.Subscriber('/cf1/imu',Imu,clbk_imu)
    # imu_pub_ekf = rospy.Publisher('/imu_data', Imu, queue_size=1)  
    imu_pub_comple_filter = rospy.Publisher('/imu_data', Imu, queue_size=1) 

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()