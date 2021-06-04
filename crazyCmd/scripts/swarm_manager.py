#! /usr/bin/env python3
# ROS modules
import rospy

# Crazyflie modules
import cflib

class SwarmManager:
    # ==============================================================================
    #
    #                       C O N S T R U C T O R
    #
    # ==============================================================================
    def __init__(self):
        self.__drones = []


    # ==============================================================================
    #
    #                             M E T H O D S
    #
    # ==============================================================================
    def add_drone(self, drone_):
        # Check if the drone to be added to swarm is unique:
        if not self.__checkExistance(drone_):
            self.__drones.append(drone_)
        else:
            rospy.logerr("[ERROR] Crazyflie with URI %s already spawn!")

    def __checkExistance(self, drone_):
        for agent in self.__drones:
            if agent.URI == drone_.URI:
                return True
        return False