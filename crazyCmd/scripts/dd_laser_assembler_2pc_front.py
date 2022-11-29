#! /usr/bin/env python3

import rospy
from laser_assembler.srv import AssembleScans2
from sensor_msgs.msg import PointCloud2

rospy.init_node("laser_assembler_2pc_front")
rospy.wait_for_service("assemble_scans2")
assemble_scans = rospy.ServiceProxy("assemble_scans2", AssembleScans2)
pub  = rospy.Publisher("/laser_pointcloud_front", PointCloud2, queue_size=1)

r = rospy.Rate(1)

while (True):
    try:
        resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
        print("Got cloud with %u points" % len(resp.cloud.data))
        pub.publish(resp.cloud)

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    r.sleep()