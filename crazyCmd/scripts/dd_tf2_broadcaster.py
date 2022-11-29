#! /usr/bin/env python3

# this is a tranform code, which makes a relative frame, world and crazyflie_main_body, 
# in this way, we can generate a good pointcloud

import rospy

import tf

import tf2_ros
import geometry_msgs.msg

from crazyflie_messages.msg import CrazyflieState

def handle_turtle_pose(msg, turtlename):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "crazyflie_main_body"
    t.transform.translation.x = msg.position.x
    t.transform.translation.y = msg.position.y
    t.transform.translation.z = msg.position.z
    q = tf.transformations.quaternion_from_euler(msg.orientation.roll,msg.orientation.pitch,msg.orientation.yaw)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_broadcaster')
    turtlename = "cf1"
    rospy.Subscriber("/cf1/state",CrazyflieState,handle_turtle_pose,turtlename)
    rospy.spin()