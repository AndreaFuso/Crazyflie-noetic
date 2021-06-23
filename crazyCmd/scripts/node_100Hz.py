import rospy
from crazyflie_simulator.CrazySim import CrazySim
from crazyflie_manager.CrazyManager import *
from crazy_common_py.dataTypes import Vector3

if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('node_100Hz', log_level=rospy.DEBUG)

    rate = rospy.Rate(100)

    trajectory_pub = rospy.Publisher()

    while not rospy.is_shutdown():

        rate.sleep()

    rospy.spin()