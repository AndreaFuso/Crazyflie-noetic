#! /usr/bin/env python3
import rospy

if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('word_formation_node', log_level=rospy.INFO)

    rospy.spin()