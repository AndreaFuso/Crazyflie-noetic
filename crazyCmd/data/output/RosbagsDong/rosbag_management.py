#! /usr/bin/env python3



import bagpy
from bagpy import bagreader
import pandas as pd

b = bagreader('comple_filter_hand1.bag')
rpy_msg = b.message_by_topic('/cf1/rpy_comple_filter')
rpy_msg
df_rpy = pd.read_csv(rpy_msg)
df_rpy

# import rosbag
# bag = rosbag.Bag('~/catkin_ws_p/src/crazyCmd/data/output/RosbagsDong/comple_filter_hand1.bag')
# for topic, msg, t in bag.read_messages(topics=['/cf1/rpy_comple_filter']):
#     print(msg)
# bag.close()