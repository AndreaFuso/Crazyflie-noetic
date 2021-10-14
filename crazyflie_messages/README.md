# crazyflie_messages
This ros packages collects all defined messages used by all topics, actions and services.
## Messages
### Attitude.msg
```yaml
crazyflie_messages/RollPitchYaw desired_attitude
float64 desired_thrust
```
this message brings information about orientation (roll, pitch and yaw) and thrust; it WAS used by 
[MotionCommanderSim](https://github.com/AndreaFuso/Crazyflie-noetic/tree/main/crazy_common_py/src) to publish a desired 
orientation and thrust value to the **MotorControllerSim** for initial debugging operations.

### CrazyflieState.msg
```yaml
# Position of the Crazyflie in [m]
geometry_msgs/Vector3 position
crazyflie_messages/RollPitchYaw orientation

# Linear velocity:
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 rotating_speed
```
this message is used to share information about the state of the crazyflie: position [m], orientation [rad], linear 
velocity [m/s] and angular velocity [rad/s].

### CustomTrajectoryPointYaw_msg.msg
OLD
### Position.msg
```yaml
geometry_msgs/Vector3 desired_position
geometry_msgs/Vector3 desired_velocity
float64 desired_yaw
```
this message is used to deliver a reference to the flight controller: to reach a certain position, to move in a certain 
direction with given velocity components and with a given reference yaw [deg].
### RollPitchYaw.msg
```yaml
float64 roll
float64 pitch
float64 yaw
```
this message is used to share roll, pitch and yaw values [rad].

---
## Action messages
### Destination3D.action
```yaml
# GOAL SECTION
crazyflie_messages/Position destination_info
float64 time_duration
---
# RESULT SECTION
bool result
---
# FEEDBACK SECTION
float64 remaining_time
```

### Takeoff.action
```yaml
# GOAL SECTION
float64 takeoff_height
---
# RESULT SECTION
bool result
---
# FEEDBACK SECTION
float64 absolute_distance
```

### VelocityTrajectory.action
```yaml
# GOAL SECTION
crazyflie_messages/Position[] states_vector
float64 dt
---
# RESULT SECTION
bool result
---
# FEEDBACK SECTION
float64 remaining_time
```

---
## Service messages
### CustomTrajectoryPointYaw_srv.srv
```yaml
crazyflie_messages/CustomTrajectoryPointYaw_msg trajectory_point
---
bool result
```

### DesiredPosition_srv.srv
```yaml
# INPUTS
# Desired position:
geometry_msgs/Vector3 desired_position

---

# OUTPUTS
# Correct execution check:
bool response_status

# Thrust command:
float64 thrust

# Desired attitude:
crazyflie_messages/RollPitchYaw desired_attitude
```
### DesiredVelocity_srv.srv
```yaml
# INPUTS
# Desired velocity:
geometry_msgs/Vector3 desired_velocity

---

# OUTPUTS
# Correct execution check:
bool response_status

# Thrust command:
float64 thrust

# Desired attitude:
crazyflie_messages/RollPitchYaw desired_attitude
```

### MotorCommand_srv.srv
```yaml
# INPUTS
# Orientation output (roll, pitch and yaw):
crazyflie_messages/RollPitchYaw rpy_output
float64 thrust

---

# OUTPUTS
bool result
```

### Takeoff_srv.srv
```yaml
float64 takeoff_height
---
bool result
```

