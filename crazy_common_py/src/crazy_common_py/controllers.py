import math
from enum import Enum
from crazy_common_py.common_functions import constrain, sameSign


class WindupType:
    Clamped = 0
    Exclusion = 1

class WindupInfo:
    def __init__(self, lowerValueSat=0, upperValueSat=0):
        self.lowerValueSat = lowerValueSat
        self.upperValueSat = upperValueSat

class PidController:
    def __init__(self, Kp, Ki, Kd, dt, windupType, integralLimit=0, windup_info=WindupInfo()):
        # Setting up the gains:
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd

        # Setting up time increment:
        self.dt = dt

        # Setting up windup check:
        self.windupType = windupType
        self.integralLimit = integralLimit
        self.windup_info = windup_info

        # Setting up the actual and past error value:
        self.error = 0
        self.prevError = 0
        self.deltaError = 0

        # Setting up contributions:
        self.proportional = 0
        self.integral = 0
        self.derivative = 0


    def updatePID(self, actual, desired, updateError=True):
        # Calculating actual error:
        '''
        e(t) = desiredValue - actualValue
        '''
        if updateError:
            self.error = desired - actual

        self.deltaError = self.error - self.prevError

        # Proportional contribution:
        '''
        ProportionalContribution = Kp * e(t)
        '''
        self.proportional = self.kp * self.error

        # Derivative contribution:
        '''
                                       e(t_{n}) - e(t_{n-1}) 
        DerivativeContribution = Kd * -----------------------
                                                dt
        '''
        self.derivative = self.kd * (self.deltaError / self.dt)

        # Integral contribution:
        '''
        IntegralContribution = Ki * \int_{0}^{t} e(t) * dt
        '''
        self.integral = self.integral + self.ki * (self.error * self.dt)

        # Preventing windup problem:
        if self.windupType == WindupType.Clamped:
            if self.integralLimit != 0:
                self.integral = constrain(self.integral, - self.integralLimit, self.integralLimit)
        elif self.windupType == WindupType.Exclusion:
            # Checking if saturation occured:
            beforeClamping = self.proportional + self.integral + self.derivative
            afterClamping = constrain(beforeClamping, self.windup_info.lowerValueSat, self.windup_info.upperValueSat)

            # Checking sign of the error and sign of the PID output:
            if beforeClamping != afterClamping and sameSign(beforeClamping, self.error):
                self.integral = 0

        # Updating previous error value:
        self.prevError = self.error

        return self.proportional + self.integral + self.derivative

    def reset(self):
        self.proportional = 0
        self.integral = 0
        self.derivative = 0

        self.error = 0
        self.prevError = 0