import rospkg
from crazy_common_py.dataTypes import Vector3


class LaunchFileGenerator:
    def __init__(self, side, filename_in, filename_out):
        self.side = side
        rospack = rospkg.RosPack()

        self.txt_path = rospack.get_path('crazyCmd') + '/data/input/launch_info/' + filename_in
        self.launch_path = rospack.get_path('crazyCmd') + '/launch/' + filename_out


        self.launchfile = open(self.launch_path, 'w')

        self.generateLaunchFile()

        self.launchfile.close()

    def generateLaunchFile(self):
        self.initialPart()
        self.gazeboScene()
        self.paceNodes()
        self.crazyflies()
        self.finalPart()

    def initialPart(self):
        self.launchfile.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        self.launchfile.write('<launch>\n')

    def gazeboScene(self):
        self.launchfile.write('\t<!-- Launching the Gazebo world scene -->\n')
        self.launchfile.write('\t<include file="$(find gazebo_ros)/launch/empty_world.launch">\n')
        self.launchfile.write('\t\t<arg name="world_name" value="$(find crazyflie_gazebo)/worlds/crazyflie.world"/>\n')
        self.launchfile.write('\t\t<!-- more default parameters can be changed here -->\n')
        self.launchfile.write('\t\t<arg name="paused" value="true"/>\n')
        self.launchfile.write('\t</include>\n')

    def paceNodes(self):
        self.launchfile.write('\n\t<!-- Starting up nodes that give the pace to the inner controllers -->\n')
        self.launchfile.write('\t<node name="node_100Hz" pkg="crazyCmd" type="node_100Hz.py" output="screen"/>\n')
        self.launchfile.write('\t<node name="node_500Hz" pkg="crazyCmd" type="node_500Hz.py" output="screen"/>\n')

    def crazyflies(self):
        actual_pos = Vector3(0.0, 0.0, 0.2)
        cf_count = 0
        for ii in range(0, self.side):
            for jj in range(0, self.side):
                self.launchfile.write('\n\t<group ns = "cf' + str(cf_count + 1) + '">\n')
                self.launchfile.write('\t\t<node pkg="crazyCmd" type="crazyflie_spawner_node.py" name="crazyflie_spawner_node" output="screen">\n')
                self.launchfile.write('\t\t\t<rosparam param="name">cf' + str(cf_count + 1) + '</rosparam>\n')
                self.launchfile.write('\t\t\t<rosparam param="initial_position">[' + str(actual_pos.x) + ', ' + str(actual_pos.y) + ', ' + str(actual_pos.z) + ']</rosparam>\n')
                self.launchfile.write('\t\t</node>\n')
                self.launchfile.write('\t</group>\n')
                actual_pos.y = actual_pos.y + 1.0
                cf_count = cf_count + 1
            actual_pos.y = 0.0
            actual_pos.x = actual_pos.x + 1.0

        self.launchfile.write('\n\t<group ns = "swarm">\n')
        self.launchfile.write('\t\t<node pkg="crazyCmd" type="swarm_node.py" name="swarm_node" output="screen">\n')
        self.launchfile.write('\t\t\t<rosparam param="cfs_number">' + str(cf_count) + '</rosparam>\n')
        self.launchfile.write('\t\t</node>\n')
        self.launchfile.write('\t</group>\n')



    def finalPart(self):
        self.launchfile.write('\n</launch>')



LF = LaunchFileGenerator(7, 'swarm_settings.txt', 'my_launch.launch')
print(LF.launch_path)

