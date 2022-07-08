#! /usr/bin/env python3
import rospy
# from crazyflie_simulator.CrazySim import CrazySim
# from crazyflie_manager.CrazyManager import *
# from crazy_common_py.dataTypes import Vector3
from std_msgs.msg import Empty

if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('node_100Hz', log_level=rospy.DEBUG)
    ## DD: init_node 'node_100Hz' --> tells rospy the name of our node, it cannot start communicating with ROS master until rospy has this info
    ## DD: log_level=rospy.DEBUG --> publish log messages, here to see logdebug messages on /rosout topic

    # Rate definition for the main loop:
    rate = rospy.Rate(100.0)
    ## DD: creates a Rate(class) object rate, with its method .sleep(), which offers a convenient way for looping at the desired rate, here loop 100 times per second

    # Publisher definition:
    pace_100Hz_pub = rospy.Publisher('/pace_100Hz', Empty, queue_size=1)
    ## DD: create an instance pace_100Hz_pub
    ## DD: our node is publishing to the '/pace_100Hz' topic using the message type Empty, 
    ## DD: Empty is the class std_msgs.msg.Empty, 
    ## DD: queue_size limits the amount of queued messages if any subscriber is not receiving them fast enough

    # Continuous loop:
    while not rospy.is_shutdown():
        ## DD: check if our program should exit(eg. ctrl+C or others)
        pace_100Hz_pub.publish(Empty())
        ## DD: with .publish method, to publish Empty to '/pace_100Hz' topic
        rate.sleep()
        ## DD: with .sleep() method, sleeps just long enough to maintain the desired rate through the loop

    rospy.spin()
    ## DD: keeps our node from exiting until the node has been shutdown