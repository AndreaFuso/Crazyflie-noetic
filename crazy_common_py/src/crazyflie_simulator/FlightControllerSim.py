# ROS MODULES
import rospy

# CUSTOM MODULES
from crazyflie_simulator.filters import *
from crazyflie_simulator.pid import *
from crazyflie_simulator.stabilizer_types import *

# CONSTANTS
DT = (1.0 / POSITION_RATE)
POSITION_LPF_CUTOFF_FREQ = 20.0
POSITION_LPF_ENABLE = True

rpLimit  = 20;
rpLimitOverhead = 1.10
# Velocity maximums
xyVelMax = 1.0
zVelMax  = 1.0
velMaxOverhead = 1.10
thrustScale = 1000.0

class pidInit_s:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

class pidAxis_s:
    def __init__(self, pidObj, init, prevMode,setpoint=0, output=0):
        self.pid = pidObj
        self.init = init
        self.previousMode = prevMode
        self.setpoint = setpoint
        self.output = output

class this_s:
    def __init__(self, pidElVX, pidElVY, pidELVZ, pidElX, pidElY, pidElZ, thurstBase, thrustMin):
        self.pidVX = pidElVX
        self.pidVY = pidElVY
        self.pidVZ = pidELVZ
        self.pidX = pidElX
        self.pidY = pidElY
        self.pidZ = pidElZ
        self.thrustBase = thurstBase
        self.thrustMin = thrustMin


class FlightControllerSim:
    # ==================================================================================================================
    #
    #                                               C O N S T R U C T O R
    #
    # This class completely handle one virtual Crazyflie.
    # INPUTS:
    #
    # ==================================================================================================================
    def __init__(self):
        # this_s instantiation:
        self.this = this_s(pidAxis_s(PidObject(),pidInit_s(25.0, 1.0, 0.0),stab_mode_t.modeDisable),)
        pass

    # ==================================================================================================================
    #
    #                                   I N T E R N A L  C O N T R O L L E R S
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                           __P O S I T I O N  C O N T R O L L E R
    #
    # ------------------------------------------------------------------------------------------------------------------


    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                           __V E L O C I T Y  C O N T R O L L E R
    #
    # ------------------------------------------------------------------------------------------------------------------


    # ==================================================================================================================
    #
    #                       S U P P O R T  M E T H O D S  P O S I T I O N  C O N T R O L L E R S
    #
    # ==================================================================================================================