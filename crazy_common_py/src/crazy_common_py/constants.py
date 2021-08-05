from crazy_common_py.controllers import WindupType
# ======================================================================================================================
#
#                           S I M U L A T E D  C R A Z Y F L I E  D E F A U L T  V A L U E S
#
# ======================================================================================================================
# ----------------------------------------------------------------------------------------------------------------------
#                                           I M U  S E N S O R
# ----------------------------------------------------------------------------------------------------------------------
IMU_GAUSSIAN_NOISE_DEFAULT = 0.0
IMU_UPDATE_RATE_DEFAULT = 1000   # [Hz]

# ----------------------------------------------------------------------------------------------------------------------
#                                           P H Y S I C A L  C O N S T A N T S
# ----------------------------------------------------------------------------------------------------------------------
# Generic physics constants (used by original firmware):
SPEED_OF_LIGHT = 299792458.0
GRAVITY_MAGNITUDE = 9.81

# PI values (used by original firmware):
M_PI = 3.14159265358979323846
M_PI_F = 3.14159265358979323846
M_1_PI_F = 0.31830988618379067154
M_PI_2_F = 1.57079632679

# Mass of the Crazyflie:
CF_MASS= 0.027 # [kg]
# ----------------------------------------------------------------------------------------------------------------------
#                                      F I T T I N G  C O E F F I C I E N T S
# ----------------------------------------------------------------------------------------------------------------------
# Polyfit thrust - lift (Crazyflie Modelling Paper)
A0_THRUST_LIFT = 5.484560e-4
A1_THRUST_LIFT = 1.032633e-6
A2_THRUST_LIFT = 2.130295e-11

# Polyfit thrust - rotating speed (Crazyflie Modelling Paper)
A0_THRUST_ROTATING_SPEED = 380.8359
A1_THRUST_ROTATING_SPEED = 0.04076521

# Polyfit lift - torque (Crazyflie Modelling Paper):
A0_LIFT_TORQUE = 1.563383e-5
A1_LIFT_TORQUE = 0.005964552


# ----------------------------------------------------------------------------------------------------------------------
#                                         L I M I T  V A L U E S  P I D  O U T P U T S
# ----------------------------------------------------------------------------------------------------------------------
# Half motor command range:
INT16_MAX = 32767

# Motor command limits:
MAX_THRUST = 65535
MIN_THRUST = 20000
THRUST_BASE = 38180
thrustScale = 1000.0

# Default technique to solve windup problem in pid:
DEFAULT_WINDUP_TYPE = WindupType.Exclusion

# Maximum absolute velocities (used as saturation values for PositionController pid output):
MAX_VELOCITY_X = 1.0    # [m/s]
MAX_VELOCITY_Y = 1.0    # [m/s]
MAX_VELOCITY_Z = 1.0    # [m/s]

# Maximum absolute attitude (used as saturaration values for VelocityController pid output):
MAX_PITCH = 20  # [deg]
MAX_ROLL = 20   # [deg]

# Maximum absolute attitude rate (used as saturaration values for AttitudeController pid output):
MAX_ROLL_RATE = 40      # [deg/s]
MAX_PITCH_RATE = 40     # [deg/s]
MAX_YAW_RATE = 40       # [deg/s]

# Maximum absolute motor command (used as saturation values for AttitudeRateController pid output):
MAX_ROLL_OUTPUT = 10000 #7000
MAX_PITCH_OUTPUT = 10000 #7000
MAX_YAW_OUTPUT = 10000 #7000

# ----------------------------------------------------------------------------------------------------------------------
#                               D E F A U L T  P O S I T I O N I N G  V A L U E S
# ----------------------------------------------------------------------------------------------------------------------
# Takeoff operation:
DEFAULT_TAKEOFF_HEIGHT = 0.2    # [m]
DEFAULT_TAKEOFF_SPEED = 0.5     # [m/s]

# Landing operation:
DEFAULT_LAND_HEIGHT = 0.15      # [m]
DEFAULT_LAND_SPEED = 0.5        # [m/s]

# Forward/backward motion:
DEFAULT_FORWARD_VELOCITY = 1.0  # [m/s]
DEFAULT_BACKWARD_VELOCITY = 1.0  # [m/s]
# ----------------------------------------------------------------------------------------------------------------------
#                                   A C T U A T O R  V E L O C I T Y  P I D  V A L U E S
# ----------------------------------------------------------------------------------------------------------------------
ACTUATOR_VELOCITY_CONTROLLER_KP = 1.0
ACTUATOR_VELOCITY_CONTROLLER_KI = 1.0
ACTUATOR_VELOCITY_CONTROLLER_KD = 0.0
ACTUATOR_VELOCITY_CONTROLLER_I_CLAMP = 100.0

# ----------------------------------------------------------------------------------------------------------------------
#                                           G E N E R I C  P I D  V A L U E S
# ----------------------------------------------------------------------------------------------------------------------

DEFAULT_PID_INTEGRATION_LIMIT = 5000.0 #5000.0 0.0
DEFAULT_PID_OUTPUT_LIMIT = 0.0  #0.0 0.0
# ----------------------------------------------------------------------------------------------------------------------
#                                           P O S I T I O N  P I D  V A L U E S
# ----------------------------------------------------------------------------------------------------------------------
# Forward velocity desired:
PID_POSITION_X_KP = 2.0     #2.0 0.02
PID_POSITION_X_KI = 0.0     #0.0 0.0
PID_POSITION_X_KD = 0.0     #0.0 0.01

# Lateral velocity desired:
PID_POSITION_Y_KP = 2.0    #2.0 0.02
PID_POSITION_Y_KI = 0.0     #0.0
PID_POSITION_Y_KD = 0.0    #0.0 0.01

# Vertical velocity desired:
PID_POSITION_Z_KP = 2.0   #2.0 10.0
PID_POSITION_Z_KI = 0.5     #0.5 0.1
PID_POSITION_Z_KD = 0.0     #0.0 0.01
# ----------------------------------------------------------------------------------------------------------------------
#                                           V E L O C I T Y  P I D  V A L U E S
# ----------------------------------------------------------------------------------------------------------------------
# Fwd velocity desired VS actual fwd velocity => DESIRED PITCH
PID_VELOCITY_X_KP = 25.0    #25.0 20.0
PID_VELOCITY_X_KI = 1.0     #1.0
PID_VELOCITY_X_KD = 0.0     #0.0

# Lateral velocity desired VS actual lateral velocity => DESIRED ROLL
PID_VELOCITY_Y_KP = 25.0    #25.0 20.0
PID_VELOCITY_Y_KI = 1.0     #1.0
PID_VELOCITY_Y_KD = 0.0     #0.0

# Vertical velocity desired VS actual vertical velocity => THRUST
PID_VELOCITY_Z_KP = 25.0    #25.0 20.0
PID_VELOCITY_Z_KI = 15.0    #15.0
PID_VELOCITY_Z_KD = 0.0     #0.0 7.0
# ----------------------------------------------------------------------------------------------------------------------
#                                           A T T I T U D E  P I D  V A L U E S
# ----------------------------------------------------------------------------------------------------------------------
ROLL_KP = 10.0   # 10
ROLL_KI = 40.0   # 40
ROLL_KD = 110.0  # 100
PID_ROLL_INTEGRATION_LIMIT = 20.0   #20.0 50.0

PITCH_KP = 10.0     # 10
PITCH_KI = 40.0     # 40
PITCH_KD = 110.0    # 100
PID_PITCH_INTEGRATION_LIMIT = 20.0  #20.0 50.0

YAW_KP = 10.0     # 10
YAW_KI = 40.0     # 40
YAW_KD = 110.0    # 100
PID_YAW_INTEGRATION_LIMIT = 360.0   #360 50.0

# ----------------------------------------------------------------------------------------------------------------------
#                                     O T H E R  F I R M W A R E  P I D  V A L U E S
# ----------------------------------------------------------------------------------------------------------------------
'''
Rincontrollare tutti i controllori, angoli e velocita angolare dovrebbero essere in radianti;
    - non credo sia un problema della odom per la velcoita angolare(dovrebbe arrivare quella relativa, come l'orientation)
    - le vibrazioni potrebbero arrivare da parametri sbagliati del pid sullo yaw rate?
    - con angoli piccoli resiste, massimo 30 deg, se supero la soglia oscilla troppo e impazzisce
'''
# Desired Roll Rate VS actual Roll Rate => ROLL OUTPUT
PID_ROLL_RATE_KP = 100.0    #250 15.0 100 20 80 40
PID_ROLL_RATE_KI = 300.0    #500 30.0 500 100 100 60
PID_ROLL_RATE_KD = 150.0     #2.5 3.0 200 80 200 150
PID_ROLL_RATE_INTEGRATION_LIMIT = 33.3     #33.3

# Desired Pitch Rate VS actual Pitch Rate => PITCH OUTPUT
PID_PITCH_RATE_KP = 100.0   #250 15.0
PID_PITCH_RATE_KI = 300.0  #500 30.0
PID_PITCH_RATE_KD = 150.0    #2.5 3.0
PID_PITCH_RATE_INTEGRATION_LIMIT = 33.3 #33.3

# Desired Yaw Rate VS actual Yaw Rate => YAW OUTPUT
PID_YAW_RATE_KP = 120.0 #120 7.0 50
PID_YAW_RATE_KI = 17.0  #16.7 1.0 250
PID_YAW_RATE_KD = 0.0   #0.0 100
PID_YAW_RATE_INTEGRATION_LIMIT = 33.3  #166.7 33.3

# Desired Roll VS actual Roll => DESIRED ROLL RATE
PID_ROLL_KP = 6.0  #6.0 3.0 2.5 3.0 0.25
PID_ROLL_KI = 3.0   #3.0 3.0 1.0 1.5
PID_ROLL_KD = 0.0   #0.0 0.035 3.5 0.035

# Desired Pitch VS actual Pitch => DESIRED PITCH RATE
PID_PITCH_KP = 6.0  #6.0 3.0 0.25
PID_PITCH_KI = 3.0  #3.0 3.0 1.0 1.5
PID_PITCH_KD = 0.0  #0.0 0.035 3.5

# Desired YAW VS actual YAW => DESIRED YAW RATE
PID_YAW_KP = 6.0    #6.0 0.25
PID_YAW_KI = 1.0    #1.0 1.0
PID_YAW_KD = 0.35   #0.35 3.5
# ======================================================================================================================
#
#                               R E A L  C R A Z Y F L I E  D E F A U L T  V A L U E S
#
# ======================================================================================================================
DELAY_DECK_CHECK = 1

DEFAULT_TAKEOFF_HEIGHT = 0.2

