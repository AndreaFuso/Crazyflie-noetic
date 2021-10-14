# ======================================================================================================================
#
#                          P U B L I S H E R S  /  S U B S C R I B E R S  T O P I C S
#
# ======================================================================================================================
# ----------------------------------------------------------------------------------------------------------------------
#                                      U R D F  R E F E R E N C E  T O P I C S
# ----------------------------------------------------------------------------------------------------------------------
# "Raw" odometry coming from Gazebo:
DEFAULT_ODOMETRY_TOPIC = 'odom_absolute'

# Force state to be applied to each propeller:
DEFAULT_FORCE_STATE_TOPIC_M1 = 'force_state_M1'
DEFAULT_FORCE_STATE_TOPIC_M2 = 'force_state_M2'
DEFAULT_FORCE_STATE_TOPIC_M3 = 'force_state_M3'
DEFAULT_FORCE_STATE_TOPIC_M4 = 'force_state_M4'

# ----------------------------------------------------------------------------------------------------------------------
#                                           C R A Z Y F L I E  S T A T E
# ----------------------------------------------------------------------------------------------------------------------
# Topic where each crazyflie publishes its state:
DEFAULT_CF_STATE_TOPIC = 'state'

# ----------------------------------------------------------------------------------------------------------------------
#                                          1 0 0 H z  &  5 0 0 H z  P A C E
# ----------------------------------------------------------------------------------------------------------------------
# Topic where node_100Hz and node_500Hz are publishing their Empty messages:
DEFAULT_100Hz_PACE_TOPIC = 'pace_100Hz'
DEFAULT_500Hz_PACE_TOPIC = 'pace_500Hz'

# ----------------------------------------------------------------------------------------------------------------------
#                           S I M U L A T E D  C R A Z Y F L I E  M O T O R  C O M M A N D S
# ----------------------------------------------------------------------------------------------------------------------
# Topic through which MotionCommanderSim decides to send motor commands to MotorControllerSim:
DEFAULT_MOTOR_CMD_TOPIC = 'motor_command'

# Topic through which FlightControllerSim tells MotionCommanderSim what should send to MotorControllerSim:
DEFAULT_DESIRED_MOTOR_CMD_TOPIC = 'desired_motor_command'

# Topic through which MotionCommanderSim keeps publishing actual desired position to FlightControllerSim:
DEFAULT_ACTUAL_DESTINATION_TOPIC = 'actual_state_target'

# ======================================================================================================================
#
#                                           A C T I O N S  T O P I C S
#
# ======================================================================================================================
# ----------------------------------------------------------------------------------------------------------------------
#                                          C R A Z Y F L I E  A C T I O N S
# ----------------------------------------------------------------------------------------------------------------------
# Takeoff & landing actions:
DEFAULT_TAKEOFF_ACT_TOPIC = 'takeoff_actn'
DEFAULT_LAND_ACT_TOPIC = 'land_actn'

# Absolute position target motion:
DEFAULT_ABS_POS_TOPIC = 'absolute_pos_motion'

# Relative position displacement:
DEFAULT_REL_POS_TOPIC = 'relative_pos_motion'


# ======================================================================================================================
#
#                                           S E R V I C E S  T O P I C S
#
# ======================================================================================================================
# Takeoff & landinng services:
DEFAULT_TAKEOFF_SRV_TOPIC = 'takeoff_srv'
DEFAULT_LAND_SRV_TOPIC = 'land_srv'
