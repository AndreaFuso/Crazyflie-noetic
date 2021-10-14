# crazyflie_description
This ros package contains all the information needed to build a virtual crazyflie:
* **meshes:** all model's meshes in .dae format, located at ../crazyflie_description/meshes/ ;
* **urdf files:** where the actual instructions to build the model are written down;

## URDF files
### crazyflie_reference.urdf
This is the used *urdf file* used within the framework, it's not a "working urdf file", in fact within this document 
there are some custom tags, that are used as placeholders by 
[CrazySim](https://github.com/AndreaFuso/Crazyflie-noetic/tree/main/crazy_common_py/src) to modify some parameters
(such as the robot namespace). </br>
Going in further details, the file is composed by some sections.

### crazyflie_reference.urdf - LINKS
In the first section we have the definition of each link: </br> </br>
**MAIN BODY** </br>
Definition of the virtual crazyflie chassis and its properties (mass, inertia, exc.). </br> </br>
**IMU SENSOR LINK** </br>
This link is used to represent the location of the IMU sensor. </br> </br>
**PROPELLER MOTORS** </br>
Four links representing the propellers and their properties (mass, inertia, exc.).

### crazyflie_reference.urdf - JOINTS
In the second section we have the definitions of the joints, so how links are linked together: </br> </br>
**IMU SENSOR FIXED JOINT** </br>
This joint positions imu sensor link at a certain distance with respect crazyflie's chassis reference system, in 
correspondence of the position of actual sensor. </br> </br>
**PROPELLERS REVOLUTE JOINTS** </br>
Four revolute joints to link the propellers to the chassis.

### crazyflie_reference.urdf - GENERIC PLUGINS
In the third section we have the used plugins: </br> </br>
**GAZEBO ROS CONTROL** </br>
```xml
    <!--
    +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                        G A Z E B O  R O S  C O N T R O L
    +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->
    <gazebo>
        <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
          <robotNamespace>ROBOT_NAMESPACE</robotNamespace>
        </plugin>
    </gazebo>
```
**gazebo_ros_control** plugin is used to actuate the joints, basically lets ROS control their motion in Gazebo, creating some topics. 
*ROBOT_NAMESPACE* is an example of above discussed tags/placeholders. </br> </br>
**ODOMETRY** </br>
```xml
    <!--
    +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                                O D O M E T R Y
    +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->
    <gazebo>
        <plugin name="p3d_base_controller" filename="libgazebo_ros_p3d.so">
            <alwaysOn>true</alwaysOn>
            <updateRate>500.0</updateRate>
            <bodyName>crazyflie_main_body</bodyName> <!--crazyflie_main_body-->
            <topicName>odom_absolute</topicName>
            <gaussianNoise>0.0</gaussianNoise>
            <frameName>world</frameName> <!-- world -->
            <xyzOffsets>0 0 0</xyzOffsets>
            <rpyOffsets>0 0 0</rpyOffsets>
        </plugin>
    </gazebo>
```
using **p3d_base_controller** plugin a topic (by the name specified in *topicName* tag) is created, publishing at 
500Hz (*updateRate* tag); at current version, measurements are related to *bodyName* (chassis) with respect world frame, 
since the sensor (IMU) is not simulated. Moreover *gaussianNoise* is equal to 0.0, since 
[FakeStateEstimator](https://github.com/AndreaFuso/Crazyflie-noetic/tree/main/crazy_common_py/src)
is used. </br> </br>

### crazyflie_reference.urdf - ACTUATORS
```xml
    <!--
    +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                        A C T U A T O R  M O T O R  1
    +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->

    <transmission name="transmission_M1">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="crazyflie_M1_joint">
          <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
        </joint>
        <actuator name="M1">
          <mechanicalReduction>1</mechanicalReduction>
          <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
        </actuator>

    </transmission>
```
in the above snippet is possible to see the actuator related to motor one, it can be defined by *transmission* tag, 
available thanks to **gazebo_ros_control** plugin. Each transmission is defined by:
* **type:** type of the transmission, in this case *SimpleTransmission*;
* **joint:** joint's name to be actuated, and type of actuating joint, in this case *VelocityJointInterface*, since the control is performed in velocity;
* **actuator:** virtual motor, since *VelocityJointInterface* is used, a PID is set up, its parameters have to be properly set using *rosparameters* (this is automatically done by **CrazysSim**); 

### crazyflie_reference.urdf - FORCE PLUGINS
```xml
    <!--
    +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                    F O R C E / T O R Q U E  M O T O R  1
    +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->

    <gazebo>
        <plugin name="gazebo_ros_force" filename="libgazebo_ros_force.so">
              <alwaysOn>true</alwaysOn>
              <update>500</update>
              <updateRate>500.0</updateRate>
              <bodyName>crazyflie_prop_M1</bodyName>
              <topicName>lift_M1</topicName>
        </plugin>
    </gazebo>
```
**gazebo_ros_force** plugin creates a topic (name specified in *topicName*) through which it is possible to set up the 
force state in correspondence of a link (*bodyName* tag), with a given update frequency in Hz (*update*/*updateRate*). 



