# Crazyflie-noetic
## Description
Crazyflie-noetic collects different [ROS](https://www.ros.org/) packages, which are all aimed to control both a real Crazyflie 2.1 and a 
simulated
one. Thanks to these packages it is also possible to control a real/simulated swarm as well. The simulation is performed
 in [Gazebo](http://gazebosim.org/), while all the framework is handled by ROS, exploiting all the advantages coming from
its architecture: nodes, topics, actions and services. Here it follows a table with tested operating systems: <br/>
Operating system | Version
---------------- | -------
Ubuntu           | 24.04.2 LTS


## Packages
There are two types of 
packages:
* **custom packages**;
* **third party packages**;

**custom packages** collect all packages created from scratch, designed to be used with ROS Noetic and Python3
(tested with Ubuntu 20.04 LTS); while **third party packages** collect some third party packages working with older 
version of ROS (like [CrazyS](https://github.com/gsilano/CrazyS)), partially modified in order to make them work with 
ROS Noetic. Third party packages have not been fully tested, since finally they have not been further developed.

## Custom packages
### crazy_common_py
This package collects all the Python modules shared within the framework: classes, data types, functions and constants.
For further details see [crazy_common_py](https://github.com/AndreaFuso/Crazyflie-noetic/tree/main/crazy_common_py/src)
README file.

### crazyCmd

### crazyflie_description

### crazyflie_messages

### crazyflie_gazebo

## Third party packages
WIP
# Installation instructions
WIP
### ROS Noetic
WIP
### ROS packages
WIP
### Gazebo
WIP
### Gazebo plugins