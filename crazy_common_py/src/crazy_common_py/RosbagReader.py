import rosbag
import rospkg

from crazy_common_py.common_functions import rad2deg

rospack = rospkg.RosPack()

bag_name = 'cf_states_6_drones.bag'
bag_path = rospack.get_path('crazyCmd') + '/data/output/Rosbags/' + bag_name
bag = rosbag.Bag(bag_path)

roll = []
pitch = []
yaw = []
time_sec = []
time_nsec = []

for topic, msg, t in bag.read_messages(topics=['/cf1/state']):
    roll.append(rad2deg(msg.orientation.roll))
    pitch.append(rad2deg(msg.orientation.pitch))
    yaw.append(rad2deg(msg.orientation.yaw))

    time_sec.append(t.secs)
    time_nsec.append(t.nsecs)

print(roll)

bag.close()