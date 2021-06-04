#! /usr/bin/env python3
import rospy
from crazy_common_py.dataTypes import prova

if __name__ =='__main__':
    rospy.init_node('nodo_prova')
    prova()
    rospy.spin()
