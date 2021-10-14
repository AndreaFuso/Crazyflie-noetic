# crazyCmd
This ros package contains:
* input data (like initial swarm formation);
* launch files;
* nodes;

## Input data
### Swarm initial formation
The information about initial swarm formation are contained within a txt file located in ../crazyCmd/data/input/launch_info/,
the rules are specified within "swarm_settings.txt". In future the spawning settings will be handled by the GUI app.

----
## Launch files
### crazy_app.launch
Launch file used to launch the node related to the GUI app.

### example.launch
Launch file used to launch the **example node**(example.py).

### my_launch.launch
Launch file created by **LaunchFileGenerator** taking as input the above discussed txt file.

### real_crazyflie_test.launch
Launch file used to launch **crazyflie_real_node**.

### all other launch files
Not used anymore, may be deleted in the future.

---
## Nodes
They are located in ../crazyCmd/scripts/

### crazy_app_node.py
This node contains the process related to the GUI app.

### crazyflie_manager.py
Old version, DO NOT USE IT!

### crazyflie_real_node.py
In this node an instance of [CrazyDrone](https://github.com/AndreaFuso/Crazyflie-noetic/tree/main/crazy_common_py/src) is 
created, in order to set up a **real crazyflie**, so each real crazyflie has it own node (and namespace).

### crazyflie_spawner_node.py
In this node an instance of [CrazySim](https://github.com/AndreaFuso/Crazyflie-noetic/tree/main/crazy_common_py/src) is 
created, in order to set up a **virtual crazyflie**, so each virtual crazyflie has its own node (and namespace).

### example.py 
This node is an early versione used to control one or multiple crazyflies: at the beginning some methods were developed 
to directly command virtual crazyflie within a script, just to easily perform test operations; finally it has been 
decided to use of course ROS framework (topics, actions and services), that's why this node is no more used and the 
correspondent [CrazySim](https://github.com/AndreaFuso/Crazyflie-noetic/tree/main/crazy_common_py/src) 's methods have 
not been further developed. This node will be probably deleted in the future.

### node_100Hz.py / node_500Hz.py
These nodes just set up a *publisher* publishing an *Empty message*, to give the pace to the inner controllers of 
the virtul flight controller.

### pyramid_swarm_node.py
This node instantiates a [CrazyPyramidSwarmSim](https://github.com/AndreaFuso/Crazyflie-noetic/tree/main/crazy_common_py/src)
object, to handle a virtual swarm in pyramid formation.

### swarm_node.py
This node instantiates a [CrazySwarmSim](https://github.com/AndreaFuso/Crazyflie-noetic/tree/main/crazy_common_py/src)
object, to handle a virtual swarm in grid formation.

### keyboard_contro.py
This node was used to spawn an instance of **KeyboardController** to control a virtual crazyflie using the keyboard, 
anyway it has not been fully developed.
