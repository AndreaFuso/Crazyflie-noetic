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

PID_ROLL_RATE_KP = 0.52    #250
PID_ROLL_RATE_KI = 0.52    #500
PID_ROLL_RATE_KD = 0.025      #2.5
PID_ROLL_RATE_INTEGRATION_LIMIT = 33.3      #33.3

PID_PITCH_RATE_KP = 0.52   #250
PID_PITCH_RATE_KI = 0.52  #500
PID_PITCH_RATE_KD = 0.025     #2.5
PID_PITCH_RATE_INTEGRATION_LIMIT = 33.3 #33.3

PID_YAW_RATE_KP = 0.52 #120
PID_YAW_RATE_KI = 0.52  #16.7
PID_YAW_RATE_KD = 0.025   #0.0
PID_YAW_RATE_INTEGRATION_LIMIT = 166.7  #166.7

PID_ROLL_KP = 3.0   #6.0
PID_ROLL_KI = 3.0   #3.0
PID_ROLL_KD = 0.035   #0.0
PID_ROLL_INTEGRATION_LIMIT = 20.0   #20.0

PID_PITCH_KP = 3.0  #6.0
PID_PITCH_KI = 3.0  #3.0
PID_PITCH_KD = 0.035  #0.0
PID_PITCH_INTEGRATION_LIMIT = 20.0  #20.0

PID_YAW_KP = 6.0    #6.0
PID_YAW_KI = 1.0    #1.0
PID_YAW_KD = 0.035   #0.35
PID_YAW_INTEGRATION_LIMIT = 360.0   #360


DEFAULT_PID_INTEGRATION_LIMIT = 5000.0
DEFAULT_PID_OUTPUT_LIMIT = 0.0
# ----------------------------------------------------------------------------------------------------------------------
#                                           P O S I T I O N  P I D  V A L U E S
# ----------------------------------------------------------------------------------------------------------------------

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