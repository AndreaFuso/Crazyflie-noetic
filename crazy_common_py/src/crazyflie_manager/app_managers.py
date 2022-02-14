from enum import Enum
from crazy_common_py.dataTypes import Vector3
from crazy_common_py.constants import DEFAULT_PYRAMID_SWARM_NAME, DEFAULT_GRID_SWARM_NAME
import math

class CrazyflieType(Enum):
    SIMULATED = 0
    REAL = 1

class ElementType(Enum):
    NODE = 0
    LAUNCHFILE = 1
    SWARM = 2
    GAZEBO = 3
    CRAZYFLIE = 4

class SwarmType(Enum):
    PYRAMID = 0
    GRID = 1

class NodeInfo:
    def __init__(self, package, node_name, script_name):
        self.package = package
        self.node_name = node_name
        self.script_name = script_name
        self.type = ElementType.NODE

class LauncFileInfo:
    def __init__(self, package, name):
        self.package = package
        self.name = name
        self.type = ElementType.LAUNCHFILE

class GazeboInfo:
    def __init__(self):
        self.type = ElementType.GAZEBO

class SwarmInfo:
    def __init__(self, cf_numbers):
        self.cf_numbers = cf_numbers
        self.type = ElementType.SWARM

class CrazyfliesInfo:
    def __init__(self, cf_numbers):
        self.cf_numbers = cf_numbers
        self.type = ElementType.CRAZYFLIE

class PreviewLaunchManager:
    def __init__(self):
        self.__lf_manager = LaunchFileManager()
        self.__elements = []
        self.__cfs_number = 1
        self.__swarm_type = SwarmType.GRID
        self.__cf_type = CrazyflieType.SIMULATED
        self.__cf_positions = []
        self.__parameters = []

    def setSwarmProperties(self, cfs_number, cf_type, swarm_type=SwarmType.GRID, parameters=[]):
        self.__cfs_number = cfs_number
        self.__cf_type = cf_type
        self.__swarm_type = swarm_type
        self.__parameters = parameters
        self.__compute_coordinates()

    def addNodeByString(self, text_info):
        package_end_pos = text_info.find(':')
        package_name = text_info[0:package_end_pos]

        node_end_pos = text_info.find(':', package_end_pos + 1)
        node_name = text_info[package_end_pos + 1:node_end_pos]

        script_name = text_info[node_end_pos + 1:]

        tmp_node = NodeInfo(package_name, node_name, script_name)
        self.addElement(tmp_node)

    def addLaunchByString(self, text_info):
        package_end_pos = text_info.find(':')
        package_name = text_info[0:package_end_pos]

        launchfile_name = text_info[package_end_pos + 1:]

        tmp_launchfile = LauncFileInfo(package_name, launchfile_name)
        self.addElement(tmp_launchfile)

    def addElementByString(self, text_info):
        if text_info.count(':') == 2:
            self.addNodeByString(text_info)
        else:
            self.addLaunchByString(text_info)

    def addElement(self, element):
        self.__elements.append(element)

    def clear(self):
        self.__elements = []
        self.__cf_positions = []

    def removeElement(self, pos):
        self.__elements.pop(pos)

    def moveElement(self, from_pos, to_pos):
        if self.__elements[from_pos].type == ElementType.NODE:
            copied_element = NodeInfo(self.__elements[from_pos].package, self.__elements[from_pos].node_name,
                                      self.__elements[from_pos].script_name)
        elif self.__elements[from_pos].type == ElementType.NODE:
            copied_element = LauncFileInfo(self.__elements[from_pos].package, self.__elements[from_pos].name)
        elif self.__elements[from_pos].type == ElementType.GAZEBO:
            copied_element = GazeboInfo()
        elif self.__elements[from_pos].type == ElementType.SWARM:
            copied_element = SwarmInfo(self.__elements[from_pos].cf_numbers)
        else:
            copied_element = self.__elements[from_pos].cf_numbers

        if from_pos > to_pos:
            self.__elements.insert(to_pos, copied_element)
            self.__elements.pop(from_pos)
        else:
            self.__elements.insert(to_pos, copied_element)
            self.__elements.pop(from_pos + 1)

    def getElements(self):
        return self.__elements[:]

    def generateFile(self, name):
        self.launchfile = open(f'../../../crazyCmd/launch/{name}.launch', 'w')

        self.__opening_section()
        self.__elements_section()
        self.__cf_section()
        self.__swarm_section()
        self.__closing_section()
        self.launchfile.close()

    def generateAndLaunch(self):
        pass

    def __compute_coordinates(self):
        self.__cf_positions = []
        if self.__swarm_type == SwarmType.GRID:
            cfs_x_side = self.__parameters[1][0]
            cfs_y_side = self.__parameters[1][1]
            x_offset = self.__parameters[1][2]
            y_offset = self.__parameters[1][3]
            spawn_altitude = self.__parameters[1][4]

            spawn_pos_x = 0.0
            spawn_pos_y = 0.0
            spawn_pos_z = spawn_altitude
            cf_cont = 0
            for row in range(0, cfs_x_side):
                for column in range(0, cfs_y_side):
                    cf_cont += 1
                    if cf_cont > self.__cfs_number:
                        break
                    else:
                        self.__cf_positions.append(Vector3(spawn_pos_x, spawn_pos_y, spawn_pos_z))
                    spawn_pos_y = spawn_pos_y + y_offset

                spawn_pos_y = 0.0
                spawn_pos_x = spawn_pos_x + x_offset

        elif self.__swarm_type == SwarmType.PYRAMID:
            levels = self.__parameters[0][0]
            drone_distance = self.__parameters[0][1]
            vertical_offset = self.__parameters[0][2]
            spawn_altitude = self.__parameters[0][3]

            # Calculating the number of drones:
            cf_per_level = [1]
            self.__cf_positions.append(Vector3(0.0, 0.0, spawn_altitude))
            for level in range(1, levels + 1):
                cf_in_level = 2 * (level + 1) + 2 * (level - 1)
                cf_per_level.append(cf_in_level)
                # Calculating first cf position (top left one, with x axis pointing down and y pointing right):
                x_pos = - drone_distance * level * 0.5
                y_pos = - drone_distance * level * 0.5
                z_pos = self.__cf_positions[-1].z
                self.__cf_positions.append(Vector3(x_pos, y_pos, z_pos))
                horizontal = True
                cont = 1
                side = 1
                for cf in range(2, cf_in_level + 1):
                    if cont > level:
                        horizontal = not horizontal
                        side += 1
                        cont = 1

                    x_pos_old = self.__cf_positions[-1].x
                    y_pos_old = self.__cf_positions[-1].y
                    z_pos_old = self.__cf_positions[-1].z

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

                    self.__cf_positions.append(Vector3(x_pos_new, y_pos_new, z_pos_new))
                    cont += 1
        self.__cfs_number = len(self.__cf_positions)


    def __opening_section(self):
        self.launchfile.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        self.launchfile.write('<launch>\n\n')

    def __closing_section(self):
        self.launchfile.write('</launch>')

    def __elements_section(self):
        for ii in range(0, len(self.__elements)):
            if self.__elements[ii].type == ElementType.NODE:
                self.__node_section(self.__elements[ii])
            elif self.__elements[ii].type == ElementType.LAUNCHFILE:
                self.__launch_section(self.__elements[ii])

    def __node_section(self, node):
        self.launchfile.write(f'\t<node name="{node.node_name}" pkg="{node.package}" type="{node.script_name+".py"}" output="screen"/>\n')

    def __launch_section(self, launch):
        pass

    def __gazebo_section(self):
        pass

    def __swarm_section(self):
        swarm_name = ''
        if self.__swarm_type == SwarmType.PYRAMID:
            swarm_name = DEFAULT_PYRAMID_SWARM_NAME
            swarm_node = 'pyramid_swarm_node'
        else:
            swarm_name = DEFAULT_GRID_SWARM_NAME
            swarm_node = 'swarm_node'
        text = f'''
	<group ns = "{swarm_name}">
		<rosparam param="cfs_number">{self.__cfs_number}</rosparam>
		<node pkg="crazyCmd" type="{swarm_node}.py" name="{swarm_node}" output="screen">
			<rosparam param="cfs_number">{self.__cfs_number}</rosparam>
		</node>
	</group>
'''
        self.launchfile.write(text)

    def __cf_section(self):
        cont = 1
        cf_type_text = ''
        if self.__cf_type == CrazyflieType.SIMULATED:
            cf_type_text = 'crazyflie_spawner_node'
        else:
            cf_type_text = 'crazyflie_real_node'
        text = ''
        for position in self.__cf_positions:
            text += f'''
    <group ns = "cf{cont}">
        <node pkg="crazyCmd" type="{cf_type_text}.py" name="{cf_type_text}" output="screen">
            <rosparam param="name">cf{cont}</rosparam>
            <rosparam param="initial_position">[{position.x}, {position.y}, {position.z}]</rosparam>
        </node>
    </group>
'''
            cont += 1
        self.launchfile.write(text)

class LaunchFileManager:
    def __init__(self):
        pass

    def writeNode(self):
        pass

    def writeLaunch(self):
        pass

