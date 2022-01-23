from enum import Enum

class CrazyflieType(Enum):
    SIMULATED = 0
    REAL = 1

class ElementType(Enum):
    NODE = 0
    LAUNCHFILE = 1
    SWARM = 2
    GAZEBO = 3
    CRAZYFLIE = 4

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

    def setSwarmProperties(self, cfs_number, cf_type):
        pass

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

    def addElement(self, element):
        self.__elements.append(element)

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

    def generateFile(self):
        pass

    def generateAndLaunch(self):
        pass


class LaunchFileManager:
    def __init__(self):
        pass

    def writeNode(self):
        pass

    def writeLaunch(self):
        pass

