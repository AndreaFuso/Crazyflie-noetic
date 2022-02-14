# crazy_common_py
## crazy_common_py
This module collects all classes, data types, exc. shared among all other modules.
### common_functions.py
As its name suggests, it contains all commonly used defined functions.
### constants.py
This script contains all constants definitions, for example:
* polynomial fitting coefficients to map motor's input command into rotating speed, thrust, exc.;
* constraint values for PIDs within simulated flight controller: maximum speed, maximum attitude, exc.;
* default values: takeoff height, velocity during takeoff, exc.;
* all <img src="https://render.githubusercontent.com/render/math?math=k_p,k_i,k_d"> values for PIDs in flight controller;

### default_topics.py
This script collects all default names used for every topic, action and service, in order to have all of them in place,
so that:
* easier to change their name if desired;
* small chances to "copy" wrong name when defining a topic/action/service;

### controllers.py
This script contains different classes definitions related to different types of controllers, like:
* PidController;

### dataTypes.py
This script contains different data types definitions.

### default_rosparameters.py
This script collects all default names of available rosparameters.

### LaunchFileGenerator.py
This script contains the definition of class *LaunchFileGenerator* which is used to interpreter a txt file (located at 
../crazyCmd/data/input/launc_info/*.txt), giving instructions about the simulation one wants to set up; then the class 
accordingly generate a launch file.

## crazyflie_drone
This module collects all classes used to handle a real crazyflie.

### CrazyDrone.py
This script contains the definition of *CrazyDrone* class, used to set up all topics, actions and services used to 
handle the real crazyflie.

## crazyflie_manager
This module contains all classes used as a bridge between real and simulated crazyflies; basically it's able to handle 
both a simulated and real crazyflie, properly setting up all topics, actions and services.

### CrazyAppManager.py (WIP)
This script contains the definition of the GUI application to control real/simulated crazyflie.
<br/>The application uses PyQt5 framework, so in order to use it, following installations are required:
```bat
pip3 install pyqt5
pip3 install qtwidgets
```
the second module is used for a particular widget.
### CrazyManager.py (WIP)
This script contains the definition of all classes used to control one real/simulated crazyflie.


### SwarmMaganer.py (WIP)
This script contains the definition of all classes used to control one swarm of real/simulated crazyflies.

---
## crazyflie_simulator
This module collects all classes used to simulate one crazyflie.

### CrazySim.py
This script contains the definition of *CrazySim* class, which is used to spawn one simulated crazyflie within Gazebo 
simulation. The spawn is done using roslaunch Python library, instead of using traditional *.launc file*; using this 
library may be not that efficient, that's why probably this spawning method will be changed. </br> 
The class performs different operations:
* reads the reference crazyflie *urdf file*, modifies some parameters (like robot name space, sensors noise, exc.) using default values (constants.py);
* creates *rosparameters* used by *VelocityJointControllers* to control the simulated propellers rotating speed;
* spawns the defined crazyflie calling a Gazebo service;
* starts Gazebo's *VelocityJointControllers*;
* instantiates:
  * **StateEstimatorSim:** used to estimate the state of the simulated crazyflie;
  * **MotionCommanderSim:** used to set up all the framework (topics, actions and services) used to command the virtual crazyflie;

### filters.py
This script contains the re-definition of some filters used within Crazyflie's firmware.

### FlightControllerSimCustom.py
This script contains the definition of the used simulated flight controller, basically it's a re-definition of the 
PIDs cascade defined in the real Crazyflie's firmware. </br>
![Crazyflie PIDs cascade](../../README_images/cascaded_pid_controller.png)
For further details see the code or [Bitcraze's webpage](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/controllers/).

### MotionCommanderSim.py
This script contains the definition of class *MotionCommanderSim*, it basically "mimics" the MotionCommander present 
in [Bycraze's Python API](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/user-guides/python_api/)
to control a real crazyflie using high level commands; in particular it contains:
* **FlightControllerSim:** to simulate the firmware's flight controller;
* **MotorControllerSim:** to "simulate" the effect of the motor within the simulation;
* low level / high level actions and services to command the simulated crazyflie;

here are reported some examples:

Class property's name | Type | Topic | Type of message | Operation
:---: | :---: | :---: | :---: | :---: 
__takeoff_act | action | /CF_NAME/takeoff_actn/goal | TakeoffAction | performs takeoff
__land_act | action | /CF_NAME/land_actn/goal | TakeoffAction | performs landing
__relative_3D_displacement_act | action | /CF_NAME/relative_pos_motion_actn/goal | Destination3DAction | performs a relative motion

### MotorControllerSim.py
This script contains the definition of *MotorSim* and *MotorControllerSim* classes:
* **MotorSim:** it simulates the effect of one single motor, once received the motor input command coming from the flight controller:
  * computes the associated propeller rotating speed;
  * computes and applies the associated lift force and drag torque;
* **MotorControllerSim**: it properly delivers motor commands to all motors;

### StateEstimatorSim.py
This script contains classes used to simulate the state estimation of the virtual crazyflie. The simplest "state estimator"
 is **FakeStateEstimator**, that basically just takes real measurements coming from Gazebo.

### pid.py / stabilizer_types.py
These scripts contain the definitions of classes handling pid controller, that has been copied from Crazyflie's firmware,
they are in "C style". They have been used for previous versions of the flight controller, since they are not used 
anymore, they might be deleted in the future.

### FlightControllerSimFirmwr.py / MyFlightControllerSim.py
Older versions of the flight controller; may be deleted in the future.

---
## crazyflie_swarm
### CrazyPyramidSwarmSim.py (BETA)
This script contains the definition of *CrazyPyramidSwarmSim* class, that handles a virtual swarm arranged in a pyramid 
formation.

### CrazySwarmSim.py (BETA)
This script contains the definition of *CrazySwarmSim* class, aimed to handle a swarm of virtual crazyflies arranged in 
a grid formation.