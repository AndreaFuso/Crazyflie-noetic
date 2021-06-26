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
SPEED_OF_LIGHT = 299792458.0
GRAVITY_MAGNITUDE = 9.81

M_PI = 3.14159265358979323846
M_PI_F = 3.14159265358979323846
M_1_PI_F = 0.31830988618379067154
M_PI_2_F = 1.57079632679

CF_MASS= 0.027 # [kg]
# ----------------------------------------------------------------------------------------------------------------------
#                                   A C T U A T O R  V E L O C I T Y  P I D  V A L U E S
# ----------------------------------------------------------------------------------------------------------------------
ACTUATOR_VELOCITY_CONTROLLER_KP = 1.0
ACTUATOR_VELOCITY_CONTROLLER_KI = 1.0
ACTUATOR_VELOCITY_CONTROLLER_KD = 0.0
ACTUATOR_VELOCITY_CONTROLLER_I_CLAMP = 100.0

# Desired Roll Rate VS actual Roll Rate => ROLL OUTPUT
PID_ROLL_RATE_KP = 0.21    #250 0.52 0.25 0.4 0.25
PID_ROLL_RATE_KI = 0.167    #500 0.52 0.8 1.5
PID_ROLL_RATE_KD = 3.0     #2.5 0.025 3.0 0.25
PID_ROLL_RATE_INTEGRATION_LIMIT = 33.3     #33.3

# Desired Pitch Rate VS actual Pitch Rate => PITCH OUTPUT
PID_PITCH_RATE_KP = 0.21   #250 0.52 0.25
PID_PITCH_RATE_KI = 0.167  #500 0.52 1.5
PID_PITCH_RATE_KD = 3.0    #2.5 0.025 0.25
PID_PITCH_RATE_INTEGRATION_LIMIT = 33.3 #33.3

# Desired Yaw Rate VS actual Yaw Rate => YAW OUTPUT
PID_YAW_RATE_KP = 0.21 #120 0.52 0.12
PID_YAW_RATE_KI = 0.167  #16.7 0.52 0.167
PID_YAW_RATE_KD = 3.0   #0.0 0.025 3.0
PID_YAW_RATE_INTEGRATION_LIMIT = 0.0  #166.7 0.7

# Desired Roll VS actual Roll => DESIRED ROLL RATE
PID_ROLL_KP = 0.25  #6.0 3.0 2.5 3.0
PID_ROLL_KI = 1.5   #3.0 3.0 1.0 1.5
PID_ROLL_KD = 0.035   #0.0 0.035 3.5 0.035
PID_ROLL_INTEGRATION_LIMIT = 20.0   #20.0

# Desired Pitch VS actual Pitch => DESIRED PITCH RATE
PID_PITCH_KP = 0.25  #6.0 3.0 0.25
PID_PITCH_KI = 1.5  #3.0 3.0 1.0
PID_PITCH_KD = 0.035  #0.0 0.035 3.5
PID_PITCH_INTEGRATION_LIMIT = 20.0  #20.0

# Desired YAW VS actual YAW => DESIRED YAW RATE
PID_YAW_KP = 0.25    #6.0 0.25
PID_YAW_KI = 1.0    #1.0 1.0
PID_YAW_KD = 3.5   #0.35 3.5
PID_YAW_INTEGRATION_LIMIT = 0.0   #360


DEFAULT_PID_INTEGRATION_LIMIT = 0.0 #5000.0
DEFAULT_PID_OUTPUT_LIMIT = 0.0  #0.0
# ----------------------------------------------------------------------------------------------------------------------
#                                           P O S I T I O N  P I D  V A L U E S
# ----------------------------------------------------------------------------------------------------------------------
# Fwd velocity desired VS actual fwd velocity => DESIRED PITCH
PID_VELOCITY_X_KP = 0.02    #25.0 0.02
PID_VELOCITY_X_KI = 0.01     #1.0 0.01
PID_VELOCITY_X_KD = 0.01     #0.0 0.01

# Lateral velocity desired VS actual lateral velocity => DESIRED ROLL
PID_VELOCITY_Y_KP = 0.02    #25.0 0.02
PID_VELOCITY_Y_KI = 0.01     #1.0 0.01
PID_VELOCITY_Y_KD = 0.01     #0.0 0.01

# Vertical velocity desired VS actual vertical velocity => THRUST
PID_VELOCITY_Z_KP = 1.0   #25.0 1.0
PID_VELOCITY_Z_KI = 0.0     #15.0
PID_VELOCITY_Z_KD = 7.0   #0.0 7.0

# Forward velocity desired:
PID_POSITION_X_KP = 0.02   #2.0 0.02
PID_POSITION_X_KI = 0.0     #0.0 0.0
PID_POSITION_X_KD = 0.01     #0.0 0.01

# Lateral velocity desired:
PID_POSITION_Y_KP = 0.02    #2.0
PID_POSITION_Y_KI = 0.0     #0.0
PID_POSITION_Y_KD = 0.01    #0.0

# Vertical velocity desired:
PID_POSITION_Z_KP = 10.0   #2.0 10.0
PID_POSITION_Z_KI = 0.1     #0.5 0.1
PID_POSITION_Z_KD = 0.01     #0.0 0.01
# ----------------------------------------------------------------------------------------------------------------------
#                                           V E L O C I T Y  P I D  V A L U E S
# ----------------------------------------------------------------------------------------------------------------------

# ----------------------------------------------------------------------------------------------------------------------
#                                           A T T I T U D E  P I D  V A L U E S
# ----------------------------------------------------------------------------------------------------------------------

# ----------------------------------------------------------------------------------------------------------------------
#                                       A T T I T U D E  R A T E  P I D  V A L U E S
# ----------------------------------------------------------------------------------------------------------------------

# ======================================================================================================================
#
#                               R E A L  C R A Z Y F L I E  D E F A U L T  V A L U E S
#
# ======================================================================================================================
DELAY_DECK_CHECK = 1

DEFAULT_TAKEOFF_HEIGHT = 0.2