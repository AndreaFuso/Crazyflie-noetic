import rospkg
from crazy_common_py.dataTypes import Vector3
from crazy_common_py.TextFormation import TextFormation
import numpy as np
import math
from enum import Enum

class SwarmType(Enum):
    GRID = 0
    PYRAMID = 1
    SENTENCE = 2

class LaunchFileGenerator:
    # ==================================================================================================================
    #
    #                                               C O N S T R U C T O R
    #
    # This class is used to generate a launch file, in order to spawn all the desired crazyflies in the simulation.
    # INPUTS:
    #   TO BE COMPLETED
    #
    # ==================================================================================================================
    def __init__(self, filename_in, filename_out):
        # Initial positions (list of Vector3):
        self.initial_positions = []

        rospack = rospkg.RosPack()

        # Assembling the path related to where the spawn info file is located:
        self.txt_path = rospack.get_path('crazyCmd') + '/data/input/launch_info/' + filename_in

        # Assembling the path related to where the launch file will be located:
        self.launch_path = rospack.get_path('crazyCmd') + '/launch/' + filename_out

        # Opening the launch file in "write mode":
        self.launchfile = open(self.launch_path, 'w')

        # Saving input file content:
        input_file = open(self.txt_path, 'r')
        self.input_file = input_file.readlines()
        input_file.close()

        # Generating the initial positions:
        self.__initial_formation_detector()

        # Generating the correct launch file spawning the whole swarm:
        self.__generate_launch_file()

        # Closing the launch file:
        self.launchfile.close()

    # ==================================================================================================================
    #
    #                                I N I T I A L  O P E R A T I O N S  M E T H O D S
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                               __ G E N E R A T E _ L A U N C H _  F I L E
    #
    # This method collects all the blocks to be written in the launch file.
    # ------------------------------------------------------------------------------------------------------------------
    def __generate_launch_file(self):
        self.__initial_part()
        self.__gazebo_scene()
        self.__pace_nodes()
        self.__crazyflies()
        self.__final_part()

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                       __ I N I T I A L _ P A R T
    #
    # This method write the header of the file.
    # ------------------------------------------------------------------------------------------------------------------
    def __initial_part(self):
        self.launchfile.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        self.launchfile.write('<launch>\n')

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                       __ G A Z E B O _ S C E N E
    #
    # This method write the block related to Gazebo, launching the background scene.
    # ------------------------------------------------------------------------------------------------------------------
    def __gazebo_scene(self):
        self.launchfile.write('\t<!-- Launching the Gazebo world scene -->\n')
        self.launchfile.write('\t<include file="$(find gazebo_ros)/launch/empty_world.launch">\n')
        self.launchfile.write('\t\t<arg name="world_name" value="$(find crazyflie_gazebo)/worlds/crazyflie.world"/>\n')
        self.launchfile.write('\t\t<!-- more default parameters can be changed here -->\n')
        self.launchfile.write('\t\t<arg name="paused" value="true"/>\n')
        self.launchfile.write('\t</include>\n')

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                       __ P A C E _ N O D E S
    #
    # This method write the block used to start the two nodes at 100Hz and 500Hz, used by inner controller of simulated
    # controllers of virtual crazyflies, in order to work at a certain frequency.
    # ------------------------------------------------------------------------------------------------------------------
    def __pace_nodes(self):
        self.launchfile.write('\n\t<!-- Starting up nodes that give the pace to the inner controllers -->\n')
        self.launchfile.write('\t<node name="node_100Hz" pkg="crazyCmd" type="node_100Hz.py" output="screen"/>\n')
        self.launchfile.write('\t<node name="node_500Hz" pkg="crazyCmd" type="node_500Hz.py" output="screen"/>\n')
        self.launchfile.write('\t<node name="node_1000Hz" pkg="crazyCmd" type="node_1000Hz.py" output="screen"/>\n')

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                       __ C R A Z Y F L I E S
    #
    # This method write the block used to spawn a certain amount of virtual crazyflies, by starting the correspondent
    # spawner node (one per each crazyflie).
    # ------------------------------------------------------------------------------------------------------------------
    def __crazyflies(self):
        cf_count = 0
        '''for ii in range(0, self.cfs_number):
            for jj in range(0, self.cfs_number):
                self.launchfile.write('\n\t<group ns = "cf' + str(cf_count + 1) + '">\n')
                self.launchfile.write('\t\t<node pkg="crazyCmd" type="crazyflie_spawner_node.py" name="crazyflie_spawner_node" output="screen">\n')
                self.launchfile.write('\t\t\t<rosparam param="name">cf' + str(cf_count + 1) + '</rosparam>\n')
                self.launchfile.write('\t\t\t<rosparam param="initial_position">[' + str(actual_pos.x) + ', ' + str(actual_pos.y) + ', ' + str(actual_pos.z) + ']</rosparam>\n')
                self.launchfile.write('\t\t</node>\n')
                self.launchfile.write('\t</group>\n')
                actual_pos.y = actual_pos.y + 1.0
                cf_count = cf_count + 1
            actual_pos.y = 0.0
            actual_pos.x = actual_pos.x + 1.0'''

        for position in range(0, self.cfs_number):
            cf_count += 1
            self.launchfile.write('\n\t<group ns = "cf' + str(cf_count) + '">\n')
            self.launchfile.write('\t\t<node pkg="crazyCmd" type="crazyflie_spawner_node.py" name="crazyflie_spawner_node" output="screen">\n')
            self.launchfile.write('\t\t\t<rosparam param="name">cf' + str(cf_count) + '</rosparam>\n')
            self.launchfile.write('\t\t\t<rosparam param="initial_position">[' + str(self.initial_positions[position].x) + ', ' + str(self.initial_positions[position].y) + ', ' + str(self.initial_positions[position].z) + ']</rosparam>\n')
            self.launchfile.write('\t\t</node>\n')
            self.launchfile.write('\t</group>\n')

        self.launchfile.write('\n\t<group ns = "swarm">\n')
        self.launchfile.write('\t\t<rosparam param="cfs_number">' + str(cf_count) + '</rosparam>\n')
        if self.__type == SwarmType.GRID or self.__type == SwarmType.SENTENCE:
            self.launchfile.write('\t\t<node pkg="crazyCmd" type="swarm_node.py" name="swarm_node" output="screen">\n')
            self.launchfile.write('\t\t\t<rosparam param="cfs_number">' + str(cf_count) + '</rosparam>\n')
        elif self.__type == SwarmType.PYRAMID:
            self.launchfile.write('\t\t<node pkg="crazyCmd" type="pyramid_swarm_node.py" name="pyramid_swarm_node" output="screen">\n')
            self.launchfile.write('\t\t\t<rosparam param="cfs_number">' + str(cf_count) + '</rosparam>\n')
            self.launchfile.write('\t\t\t<rosparam param="levels">' + str(self.__pyramid_levels) + '</rosparam>\n')
            self.launchfile.write('\t\t\t<rosparam param="vertical_offset">' + str(self.__pyramid_vertical_offset) + '</rosparam>\n')
        self.launchfile.write('\t\t</node>\n')
        self.launchfile.write('\t</group>\n')

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                       __ F I N A L _ P A R T
    #
    # This method write the final block.
    # ------------------------------------------------------------------------------------------------------------------
    def __final_part(self):
        self.launchfile.write('\n</launch>')

    # ==================================================================================================================
    #
    #                                S P A W N I N G  F O R M A T I O N  M E T H O D S
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                               __ I N I T I A L _ F O R M A T I O N _ D E T E C T O R
    #
    # This detects the user's choice of initial formation and call the correct method to generate the coordinates.
    # ------------------------------------------------------------------------------------------------------------------
    def __initial_formation_detector(self):
        # Check the initial formation that has been chosen:
        initial_formation = self.extract_value('spawn_formation', 'str')

        # Call corresponding coordinates generator:
        if initial_formation == 'grid':
            self.__grid_spawn()
            self.__type = SwarmType.GRID
        elif initial_formation == 'pyramid':
            self.__pyramid_spawn()
            self.__type = SwarmType.PYRAMID
        elif initial_formation == 'sentence':
            self.__sentence_spawn()
            self.__type = SwarmType.SENTENCE
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                       __ G R I D _ S P A W N
    #
    # This method generate initial position coordinates in grid case.
    # ------------------------------------------------------------------------------------------------------------------
    def __grid_spawn(self):
        cf_cont = 0

        # Extracting all the parameters:
        self.cfs_number = self.extract_value('number_of_cfs', 'int')
        cfs_x_side = self.extract_value('cfs_x_side', 'int')
        cfs_y_side = self.extract_value('cfs_y_side', 'int')
        x_offset = self.extract_value('x_offset', 'float')
        y_offset = self.extract_value('y_offset', 'float')
        spawn_altitude = self.extract_value('spawn_altitude', 'float')


        # Generating the coordinates:
        spawn_pos_x = 0.0
        spawn_pos_y = 0.0
        spawn_pos_z = spawn_altitude
        for row in range(0, cfs_x_side):
            for column in range(0, cfs_y_side):
                cf_cont += 1
                if cf_cont > self.cfs_number:
                    break
                else:
                    self.initial_positions.append(Vector3(spawn_pos_x, spawn_pos_y, spawn_pos_z))
                spawn_pos_y = spawn_pos_y + y_offset

            spawn_pos_y = 0.0
            spawn_pos_x = spawn_pos_x + x_offset

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                       __ P Y R A M I D _ S P A W N
    #
    # This method generate initial position coordinates in pyramid case.
    # ------------------------------------------------------------------------------------------------------------------
    def __pyramid_spawn(self):
        # Extracting all the parameters:
        levels = self.extract_value('levels', 'int')
        drone_distance = self.extract_value('drone_distance', 'float')
        vertical_offset = self.extract_value('vertical_offset', 'float')
        spawn_altitude = self.extract_value('spawn_altitude', 'float')

        # Saving values to create rosparams:
        self.__pyramid_levels = levels
        self.__pyramid_vertical_offset = vertical_offset

        # Calculating the number of drones:
        cf_per_level = [1]
        self.initial_positions.append(Vector3(0.0, 0.0, spawn_altitude))
        for level in range(1, levels + 1):
            cf_in_level = 2 * (level + 1) + 2 * (level - 1)
            cf_per_level.append(cf_in_level)
            # Calculating first cf position (top left one, with x axis pointing down and y pointing right):
            x_pos = - drone_distance * level * 0.5
            y_pos = - drone_distance * level * 0.5
            z_pos = self.initial_positions[-1].z
            self.initial_positions.append(Vector3(x_pos, y_pos, z_pos))
            horizontal = True
            cont = 1
            side = 1
            for cf in range(2, cf_in_level + 1):
                if cont > level:
                    horizontal = not horizontal
                    side += 1
                    cont = 1

                x_pos_old = self.initial_positions[-1].x
                y_pos_old = self.initial_positions[-1].y
                z_pos_old = self.initial_positions[-1].z

                if horizontal and side == 1:
                    x_pos_new = x_pos_old
                    y_pos_new = y_pos_old + drone_distance
                    z_pos_new = z_pos_old
                elif horizontal and side != 1:
                    x_pos_new = x_pos_old
                    y_pos_new = y_pos_old - drone_distance
                    z_pos_new = z_pos_old
                elif not horizontal and side == 2:
                    x_pos_new = x_pos_old + drone_distance
                    y_pos_new = y_pos_old
                    z_pos_new = z_pos_old
                elif not horizontal and side != 2:
                    x_pos_new = x_pos_old - drone_distance
                    y_pos_new = y_pos_old
                    z_pos_new = z_pos_old

                self.initial_positions.append(Vector3(x_pos_new, y_pos_new, z_pos_new))
                cont += 1
        self.cfs_number = sum(cf_per_level)

    def __sentence_spawn(self):
        sentence = "THANK YOU"
        sentence_generator_coord = TextFormation(2.0, 1.0, 1, 0.2)
        coords = sentence_generator_coord.getCoords(sentence)

        for ii in range(0, coords.shape[0]):
            self.initial_positions.append(Vector3(coords[ii, 0], coords[ii, 1], coords[ii, 2]))
        self.cfs_number = coords.shape[0]

    # ==================================================================================================================
    #
    #                    M E T H O D S  T O  G E T  I N F O S  W I T H I N  T X T  I N P U T  F I L E
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                       __ E X T R A C T _ V A L U E
    #
    # This method generate extracts a generic value from the txt, corresponding to a certain keyword.
    # INPUTS:
    #   1) keyword -> string containing the keyword of the parameter to be extracted;
    #   2) value_type -> string containing the type of the stored parameter, supported types:
    #           - str -> string
    #           - int -> integer number
    #           - float -> floating number
    # ------------------------------------------------------------------------------------------------------------------
    def extract_value(self, keyword, value_type):
        # Keyword line:
        keyword_line = 'NONE'

        # Looking for the keyword:
        for line in self.input_file:
            if keyword in line and '#' not in line:
                keyword_line = line

        # Raising error if not found:
        if keyword_line == 'NONE':
            raise Exception('ERROR: keyword "' + keyword + '" is not contained in the input file!')

        # Extracting the string containing the value:
        separation_pattern = ': '
        separation_position = keyword_line.find(separation_pattern)
        string_value = keyword_line[separation_position+len(separation_pattern):-1]

        # Casting the value:
        if value_type == 'str':
            return string_value
        elif value_type == 'int':
            return int(string_value)
        elif value_type == 'float':
            return float(string_value)
        else:
            raise Exception('ERROR: unknown data type')


LF = LaunchFileGenerator('swarm_settings.txt', 'my_launch.launch')
print('Launch file correctly created-updated at: ' + LF.launch_path)
print(LF.initial_positions[0].x, LF.initial_positions[0].y, LF.initial_positions[0].z)



