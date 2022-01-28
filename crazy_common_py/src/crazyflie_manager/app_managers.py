from enum import Enum
#from crazy_common_py.dataTypes import Vector3
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

    def setSwarmProperties(self, cfs_number, cf_type, swarm_type=SwarmType.GRID):
        self.__cfs_number = cfs_number
        self.__cf_type = cf_type
        self.__swarm_type = swarm_type


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
        self.__closing_section()
        self.launchfile.close()

    def generateAndLaunch(self):
        pass

    def __compute_coordinates(self):
        if self.__swarm_type == SwarmType.GRID:
            cfs_x_side = 1
            cfs_y_side = 1
            x_offset = 1
            y_offset = 1
            spawn_altitude = 1
        elif self.__swarm_type == SwarmType.PYRAMID:
            levels = 1
            drone_distance = 1
            vertical_offset = 1
            spawn_altitude = 1

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
        pass

    def __cf_section(self):
        pass


class LaunchFileManager:
    def __init__(self):
        pass

    def writeNode(self):
        pass

    def writeLaunch(self):
        pass

