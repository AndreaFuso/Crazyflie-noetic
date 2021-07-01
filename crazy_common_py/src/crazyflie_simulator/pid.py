# ROS MODULES
import rospy

# CUSTOM MODULES
from crazy_common_py.constants import *
from crazyflie_simulator.filters import *
from crazy_common_py.common_functions import constrain


class PidObject:
    def __init__(self, desired=0.0, kp=1, ki=1, kd=1, dt=1, enableDFilter=False, dFilter=lpf2pData(1,1,1,1,1,1,1)):
        self.error = 0
        self.prevError = 0
        self.integ = 0
        self.deriv = 0
        self.desired = desired
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.iLimit = DEFAULT_PID_INTEGRATION_LIMIT
        self.outputLimit = DEFAULT_PID_OUTPUT_LIMIT

        self.dt = dt
        self.enableDFilter = enableDFilter
        self.dFilter = dFilter

        self.outP = 0
        self.outI = 0
        self.outD = 0

def pidInit(pidObjIn, desired, kp, ki, kd, dt, samplingRate, cutoffFreq, enableFilter):
    pidObjOut = pidObjIn
    pidObjOut.error = 0
    pidObjOut.prevError = 0
    pidObjOut.integ = 0
    pidObjOut.deriv = 0
    pidObjOut.desired = desired
    pidObjOut.kp = kp
    pidObjOut.ki = ki
    pidObjOut.kd = kd
    pidObjOut.iLimit = DEFAULT_PID_INTEGRATION_LIMIT
    pidObjOut.outputLimit = DEFAULT_PID_OUTPUT_LIMIT
    pidObjOut.dt = dt
    pidObjOut.enableDFilter = enableFilter

    if pidObjOut.enableDFilter:
        pidObjOut.dFilter = lpf2pInit(pidObjOut.dFilter, samplingRate, cutoffFreq)

    return pidObjOut

def pidUpdate(pidObjIn, measured, updateError):
    pidObjOut = pidObjIn
    output = 0.0

    if updateError:
        pidObjOut.error = pidObjOut.desired - measured
    # Proportional:
    pidObjOut.outP = pidObjOut.kp * pidObjOut.error
    output = output + pidObjOut.outP

    # Derivative:
    deriv = (pidObjOut.error - pidObjOut.prevError) / pidObjOut.dt
    if pidObjOut.enableDFilter:
        result = lpf2pApply(pidObjOut.dFilter, deriv)
        pidObjOut.dFilter = result[0]
        pidObjOut.deriv = result[1]
    else:
        pidObjOut.deriv = deriv

    if math.isnan(pidObjOut.deriv):
        pidObjOut.deriv = 0

    pidObjOut.outD = pidObjOut.kd * pidObjOut.deriv
    output = output + pidObjOut.outD

    # Integral:
    pidObjOut.integ = pidObjOut.integ * pidObjOut.dt

    # Constrain the integral (unless the iLimit is zero)
    if pidObjOut.iLimit != 0:
        pidObjOut.integ = constrain(pidObjOut.integ, - pidObjOut.outputLimit, pidObjOut.outputLimit)

    pidObjOut.outI = pidObjOut.ki * pidObjOut.integ
    output = output + pidObjOut.outI

    # Constrain the total PID output (unless outputLimit is zero)
    if pidObjOut.outputLimit != 0:
        output = constrain(output, - pidObjOut.outputLimit, pidObjOut.outputLimit)

    pidObjOut.prevError = pidObjOut.error

    return (pidObjOut, output)

def pidSetIntegralLimit(pidObjIn, limit):
    pidObjOut = pidObjIn
    pidObjOut.iLimit = limit
    return pidObjOut

def pidReset(pidObjIn):
    pidObjOut = pidObjIn
    pidObjOut.error = 0
    pidObjOut.prevError = 0
    pidObjOut.integ = 0
    pidObjOut.deriv = 0
    return pidObjOut

def pidSetError(pidObjIn, error):
    pidObjOut = pidObjIn
    pidObjOut.error = error
    return pidObjOut

def pidSetDesired(pidObjIn, desired):
    pidObjOut = pidObjIn
    pidObjOut.desired = desired
    return pidObjOut

def pidGetDesired(pidObjIn):
    return pidObjIn.desired

def pidIsActive(pidObjIn):
    isActive = True
    if pidObjIn.kp < 0.0001 and pidObjIn.ki < 0.0001 and pidObjIn.kd < 0.0001:
        isActive = False
    return isActive

def pidSetKp(pidObjIn, kp):
    pidObjOut = pidObjIn
    pidObjOut.kp = kp
    return pidObjOut

def pidSetKi(pidObjIn, ki):
    pidObjOut = pidObjIn
    pidObjOut.ki = ki
    return pidObjOut

def pidSetKd(pidObjIn, kd):
    pidObjOut = pidObjIn
    pidObjOut.kd = kd
    return pidObjOut

def pidSetDt(pidObjIn, dt):
    pidObjOut = pidObjIn
    pidObjOut.dt = dt
    return pidObjOut

