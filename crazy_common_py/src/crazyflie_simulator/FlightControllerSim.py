# ROS MODULES
import time

import rospy

# CUSTOM MODULES
from crazyflie_simulator.filters import *
from crazyflie_simulator.pid import *
from crazyflie_simulator.stabilizer_types import *
from crazy_common_py.dataTypes import CfState
from crazy_common_py.common_functions import rad2deg, deg2rad

# SERVICE MESSAGES
from crazyflie_messages.srv import DesiredPosition_srv, DesiredPosition_srvResponse, DesiredPosition_srvRequest
from crazyflie_messages.srv import DesiredVelocity_srv, DesiredVelocity_srvResponse, DesiredVelocity_srvRequest

# TOPIC MESSAGES
from crazyflie_messages.msg import RollPitchYaw, CrazyflieState, Attitude, Position
from std_msgs.msg import Empty

# CONSTANTS POSITION & VELOCITY CONTROLLERS
DT = (1.0 / POSITION_RATE)
POSITION_LPF_CUTOFF_FREQ = 20.0
POSITION_LPF_ENABLE = True

rpLimit  = 20   #20
rpLimitOverhead = 1.10
# Velocity maximums
xyVelMax = 1.0
zVelMax  = 1.0
velMaxOverhead = 1.10
thrustScale = 1000.0

# CONSTANTS ATTITUDE CONTROLLER
ATTITUDE_LPF_CUTOFF_FREQ = 15.0
ATTITUDE_LPF_ENABLE = False
ATTITUDE_RATE_LPF_CUTOFF_FREQ = 30.0
ATTITUDE_RATE_LPF_ENABLE = False

ATTITUDE_CONTROLLER_FREQ = 500
ATTITUDE_CONTROLLER_DT = 1 / ATTITUDE_CONTROLLER_FREQ

# NUMBERS
INT16_MAX = 32767
MAX_THRUST = 65535



# ======================================================================================================================
#
#                                               T Y P E S  D E F I N I T I O N S
#
# ======================================================================================================================

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
        self.dt = DT


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

class control_s:
    def __init__(self, roll=0, pitch=0, yaw=0, thrust=0):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.thrust = thrust

# ======================================================================================================================
#
#                                               F U N C T I O N S
#
# ======================================================================================================================
def saturateSignedInt16(input):
    if input > INT16_MAX:
        return INT16_MAX
    elif input < -INT16_MAX:
        return -INT16_MAX
    else:
        return input

# ======================================================================================================================
#
#                                               C L A S S E S
#
# ======================================================================================================================

class FlightControllerSim:
    # ==================================================================================================================
    #
    #                                               C O N S T R U C T O R
    #
    # This class completely handle one virtual Crazyflie.
    # INPUTS:
    #
    # ==================================================================================================================
    def __init__(self, name):
        # this_s instantiation:
        self.this = this_s(pidAxis_s(PidObject(),pidInit_s(25.0, 1.0, 0.0),stab_mode_t.modeAbs),
                           pidAxis_s(PidObject(), pidInit_s(25.0, 1.0, 0.0), stab_mode_t.modeAbs),
                           pidAxis_s(PidObject(), pidInit_s(25.0, 15.0, 0.0), stab_mode_t.modeAbs),
                           pidAxis_s(PidObject(), pidInit_s(2.0, 0.0, 0.0), stab_mode_t.modeAbs),
                           pidAxis_s(PidObject(), pidInit_s(2.0, 0.0, 0.0), stab_mode_t.modeAbs),
                           pidAxis_s(PidObject(), pidInit_s(2.0, 0.5, 0.0), stab_mode_t.modeAbs),
                           36000, 20000)
        self.name = name

        # Initializing the position controller:
        self.__positionControllerInit()

        # Initializing attitude and attitude rate controllers:
        self.__attitudeControllerInit(ATTITUDE_CONTROLLER_DT)

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                           S U B S C R I B E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        self.pace_100Hz_sub = rospy.Subscriber('/pace_100Hz', Empty, self.__pace_100Hz_callback)
        self.pace_500Hz_sub = rospy.Subscriber('/pace_500Hz', Empty, self.__pace_500Hz_callback)

        # Subscriber to get the actual state of the drone in the simulation:
        state_sub = rospy.Subscriber('/' + name + '/state', CrazyflieState, self.__state_sub_callback)
        self.actual_state = CrazyflieState()


        # Subscriber looking for a point to go:
        self.desired_position_sub = rospy.Subscriber('/' + name + '/set_destination_position', Position,
                                                     self.__desired_position_sub_callback)

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                           P U B L I S H E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #self.desired_attitude_pub = rospy.Publisher('/' + name + '/set_desired_attitude', Attitude)

        self.motor_command_pub = rospy.Publisher('/' + name + '/set_desired_motor_command', Attitude, queue_size=1)
        self.desired_motor_command = Attitude()

        #self.desired_attitude_attitude_pub = rospy.Publisher('/' + name + '/desired_attitude', Attitude, queue_size=1)
        self.desired_attitude_msg = Attitude()
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                           S E R V I C E S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


    # ==================================================================================================================
    #
    #                                               C A L L B A C K S
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                           S E R V I C E S  C A L L B A C K S
    #
    # ------------------------------------------------------------------------------------------------------------------


    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                           S U B S C R I B E R S  C A L L B A C K S
    #
    # ------------------------------------------------------------------------------------------------------------------
    def __pace_100Hz_callback(self, msg):
        pass

    def __pace_500Hz_callback(self, msg):
        # Reading actual state:
        actual_state = self.actual_state

        # Reading command coming from PositionController/VelocityController:
        desired_attitude = self.desired_attitude_msg

        # Calling attitudeController:
        attitudeRateResult = self.__attitudeControllerCorrectAttitudePID(actual_state.orientation.roll,
                                                                         actual_state.orientation.pitch,
                                                                         actual_state.orientation.yaw,
                                                                         desired_attitude.desired_attitude.roll,
                                                                         desired_attitude.desired_attitude.pitch,
                                                                         desired_attitude.desired_attitude.yaw)
        # Calling attitudeRateController():
        outputResult = self.__attitudeControllerCorrectRatePID(actual_state.rotating_speed.x,
                                                               actual_state.rotating_speed.y,
                                                               actual_state.rotating_speed.z,
                                                               attitudeRateResult[0],
                                                               attitudeRateResult[1],
                                                               attitudeRateResult[2])
        '''print('Desired attitude rate: [', attitudeRateResult[0], '; ', attitudeRateResult[1], '; ',
              attitudeRateResult[2], ']')
        print('Actual attitude rate: [', actual_state.rotating_speed.x, ';', actual_state.rotating_speed.y, '; ',
              actual_state.rotating_speed.z, ']')
        print(' ')
        print('OUTPUT: [', outputResult[0], '; ', outputResult[1], '; ', outputResult[2], ']')'''

        # Setting up messaghe to be published:
        self.desired_motor_command.desired_attitude.roll = - outputResult[0]
        self.desired_motor_command.desired_attitude.pitch = - outputResult[1]
        self.desired_motor_command.desired_attitude.yaw = outputResult[2]
        self.desired_motor_command.desired_thrust = desired_attitude.desired_thrust

        # Publishing motor command:
        self.motor_command_pub.publish(self.desired_motor_command)



    def __state_sub_callback(self, msg):
        self.actual_state.position.x = msg.position.x
        self.actual_state.position.y = msg.position.y
        self.actual_state.position.z = msg.position.z

        self.actual_state.velocity.x = msg.velocity.x
        self.actual_state.velocity.y = msg.velocity.y
        self.actual_state.velocity.z = msg.velocity.z

        self.actual_state.orientation.roll = rad2deg(msg.orientation.roll)
        self.actual_state.orientation.pitch = rad2deg(msg.orientation.pitch)
        self.actual_state.orientation.yaw = rad2deg(msg.orientation.yaw)

        self.actual_state.rotating_speed.x = rad2deg(msg.rotating_speed.x)
        self.actual_state.rotating_speed.y = rad2deg(msg.rotating_speed.y)
        self.actual_state.rotating_speed.z = rad2deg(msg.rotating_speed.z)

    def __desired_position_sub_callback(self, msg):
        actual_state = self.actual_state
        thrust = 0
        attitude = attitude_t()
        setpoint = setpoint_t(position=Vector3(msg.desired_position.x, msg.desired_position.y, msg.desired_position.z),
                              mode=mode(x=stab_mode_t.modeAbs, y=stab_mode_t.modeAbs, z=stab_mode_t.modeAbs))
        state = state_t(position=Vector3(actual_state.position.x, actual_state.position.y,
                                         actual_state.position.z))
        '''print('Desired position: [', msg.desired_position.x, '; ', msg.desired_position.y, '; ', msg.desired_position.z, ']')
        print('Actual position: [', actual_state.position.x, '; ', actual_state.position.y, '; ', actual_state.position.z,']')
        print(' ')'''
        # Calling positionController():
        attitudeResult = self.__positionController(thrust, attitude, setpoint, state)
        thrust = attitudeResult[1]
        '''print('Desired attitude: [', attitudeResult[0].roll, '; ', attitudeResult[0].pitch, '; ',attitudeResult[0].yaw, ']')
        print('Actual attitude: [', actual_state.orientation.roll, '; ', actual_state.orientation.pitch, '; ', actual_state.orientation.yaw, ']')
        print(' ')'''

        self.desired_attitude_msg.desired_attitude.roll = attitudeResult[0].pitch
        self.desired_attitude_msg.desired_attitude.pitch = attitudeResult[0].roll
        self.desired_attitude_msg.desired_attitude.yaw = attitudeResult[0].yaw
        self.desired_attitude_msg.desired_thrust = thrust



        '''# Calling attitudeController:
        attitudeRateResult = self.__attitudeControllerCorrectAttitudePID(actual_state.orientation.roll,
                                                    actual_state.orientation.pitch,
                                                    actual_state.orientation.yaw,
                                                    attitudeResult[0].roll,
                                                    attitudeResult[0].pitch,
                                                    attitudeResult[0].yaw)
        # Calling attitudeRateController():
        outputResult = self.__attitudeControllerCorrectRatePID(actual_state.rotating_speed.x,
                                                               actual_state.rotating_speed.y,
                                                               actual_state.rotating_speed.z,
                                                               attitudeRateResult[0],
                                                               attitudeRateResult[1],
                                                               attitudeRateResult[2])
        print('Desired attitude rate: [', attitudeRateResult[0], '; ', attitudeRateResult[1], '; ', attitudeRateResult[2], ']')
        print('Actual attitude rate: [', actual_state.rotating_speed.x, ';', actual_state.rotating_speed.y, '; ', actual_state.rotating_speed.z, ']')
        print(' ')
        print('OUTPUT: [', outputResult[0], '; ', outputResult[1], '; ', outputResult[2], ']')
        # Setting up messaghe to be published:
        self.desired_motor_command.desired_attitude.roll = outputResult[0]
        self.desired_motor_command.desired_attitude.pitch = outputResult[1]
        self.desired_motor_command.desired_attitude.yaw = outputResult[2]
        self.desired_motor_command.desired_thrust = thrust

        # Publishing motor command:
        self.motor_command_pub.publish(self.desired_motor_command)
        #print(self.desired_motor_command)
        print(' ')
        print(' ')
        print(' ')'''





    # ==================================================================================================================
    #
    #                                       S E R V I C E S  C A L L B A C K S
    #
    # ==================================================================================================================

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
    def __positionControllerInit(self):
        self.this.pidX.pid = pidInit(self.this.pidX.pid, self.this.pidX.setpoint, self.this.pidX.init.kp,
                                     self.this.pidX.init.ki, self.this.pidX.init.kd, self.this.pidX.dt, POSITION_RATE,
                                     POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE)
        self.this.pidY.pid = pidInit(self.this.pidY.pid, self.this.pidY.setpoint, self.this.pidY.init.kp,
                                     self.this.pidY.init.ki, self.this.pidY.init.kd, self.this.pidY.dt, POSITION_RATE,
                                     POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE)
        self.this.pidZ.pid = pidInit(self.this.pidZ.pid, self.this.pidZ.setpoint, self.this.pidZ.init.kp,
                                     self.this.pidZ.init.ki, self.this.pidZ.init.kd, self.this.pidZ.dt, POSITION_RATE,
                                     POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE)

        self.this.pidVX.pid = pidInit(self.this.pidVX.pid, self.this.pidVX.setpoint, self.this.pidVX.init.kp,
                                     self.this.pidVX.init.ki, self.this.pidVX.init.kd, self.this.pidVX.dt, POSITION_RATE,
                                     POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE)
        self.this.pidVY.pid = pidInit(self.this.pidVY.pid, self.this.pidVY.setpoint, self.this.pidVY.init.kp,
                                     self.this.pidVY.init.ki, self.this.pidVY.init.kd, self.this.pidVY.dt, POSITION_RATE,
                                     POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE)
        self.this.pidVZ.pid = pidInit(self.this.pidVZ.pid, self.this.pidVZ.setpoint, self.this.pidVZ.init.kp,
                                     self.this.pidVZ.init.ki, self.this.pidVZ.init.kd, self.this.pidVZ.dt, POSITION_RATE,
                                     POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE)

    def __positionController(self, thrust, attitude, setpoint, state):
        self.this.pidX.pid.outputLimit = xyVelMax * velMaxOverhead
        self.this.pidY.pid.outputLimit = xyVelMax * velMaxOverhead
        self.this.pidZ.pid.outputLimit = max(zVelMax, 0.5) * velMaxOverhead

        cosyaw = math.cos(state.attitude.yaw * M_PI / 180.0)
        sinyaw = math.sin(state.attitude.yaw * M_PI / 180.0)
        bodyvx = setpoint.velocity.x
        bodyvy = setpoint.velocity.y

        if setpoint.mode.x == stab_mode_t.modeAbs:
            res = self.__runPid(state.position.x, self.this.pidX, setpoint.position.x, DT)
            self.this.pidX.pid = res[0]
            setpoint.velocity.x = res[1]
            #setpoint.velocity.x = self.runPid(state.position.x, self.this.pidX, setpoint.position.x, DT)
        elif setpoint.velocity_body:
            setpoint.velocity.x = bodyvx * cosyaw - bodyvy * sinyaw

        if setpoint.mode.y == stab_mode_t.modeAbs:
            res = self.__runPid(state.position.y, self.this.pidY, setpoint.position.y, DT)
            self.this.pidY.pid = res[0]
            setpoint.velocity.y = res[1]
            #setpoint.velocity.y = self.runPid(state.position.y, self.this.pidY, setpoint.position.y, DT)
        elif setpoint.velocity_body:
            setpoint.velocity.y = bodyvy * cosyaw + bodyvx * sinyaw

        if setpoint.mode.z == stab_mode_t.modeAbs:
            res = self.__runPid(state.position.z, self.this.pidZ, setpoint.position.z, DT)
            self.this.pidZ.pid = res[0]
            setpoint.velocity.z = res[1]
            #setpoint.velocity.z = self.runPid(state.position.z, self.this.pidZ, setpoint.position.z, DT)

        return self.__velocityController(thrust, attitude, setpoint, state)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                           __V E L O C I T Y  C O N T R O L L E R
    #
    # ------------------------------------------------------------------------------------------------------------------
    def __velocityController(self, thrust, attitude, setpoint, state):
        self.this.pidVX.pid.outputLimit = rpLimit * rpLimitOverhead
        self.this.pidVY.pid.outputLimit = rpLimit * rpLimitOverhead
        self.this.pidVZ.pid.outputLimit = MAX_THRUST / 2 / thrustScale

        resRoll = self.__runPid(state.velocity.x, self.this.pidVX, setpoint.velocity.x, DT)
        self.this.pidVX.pid = resRoll[0]
        rollRaw = resRoll[1]

        resPitch = self.__runPid(state.velocity.y, self.this.pidVY, setpoint.velocity.y, DT)
        self.this.pidVY.pid = resRoll[0]
        pitchRaw = resPitch[1]
        #rollRaw = self.runPid(state.velocity.x, self.this.pidVX, setpoint.velocity.x, DT)
        #pitchRaw = self.runPid(state.velocity.y, self.this.pidVY, setpoint.velocity.y, DT)
        yawRad = state.attitude.yaw * M_PI / 180

        attitude.pitch = - (rollRaw * math.cos(yawRad)) - (pitchRaw * math.sin(yawRad))
        attitude.roll = - (pitchRaw * math.cos(yawRad)) + (rollRaw * math.sin(yawRad))

        attitude.roll = constrain(attitude.roll, -rpLimit, rpLimit)
        attitude.pitch = constrain(attitude.pitch, -rpLimit, rpLimit)

        resThrust = self.__runPid(state.velocity.z, self.this.pidVZ, setpoint.velocity.z, DT)
        self.this.pidVZ.pid = resThrust[0]
        thrustRaw = resThrust[1]
        #thrustRaw = self.runPid(state.velocity.z, self.this.pidVZ, setpoint.velocity.z, DT)
        thrust = thrustRaw * thrustScale + self.this.thrustBase

        if thrust < self.this.thrustMin:
            thrust = self.this.thrustMin

        return (attitude, thrust)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                   __A T T I T U D E  C O N T R O L L E R
    #
    # ------------------------------------------------------------------------------------------------------------------
    def __attitudeControllerInit(self, updateDt):
        # Properties creation:
        self.__pidRollRate = PidObject()
        self.__pidPitchRate = PidObject()
        self.__pidYawRate = PidObject()
        self.__pidRoll = PidObject()
        self.__pidPitch = PidObject()
        self.__pidYaw = PidObject()

        # Attitude rate pids initialization:
        self.__pidRollRate = pidInit(self.__pidRollRate, 0, PID_ROLL_RATE_KP, PID_ROLL_RATE_KD, PID_ROLL_RATE_KI,
                                     updateDt, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE)
        self.__pidPitchRate = pidInit(self.__pidPitchRate, 0, PID_PITCH_RATE_KP, PID_PITCH_RATE_KD, PID_PITCH_RATE_KI,
                                      updateDt, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE)
        self.__pidYawRate = pidInit(self.__pidYawRate, 0, PID_YAW_RATE_KP, PID_YAW_RATE_KD, PID_YAW_RATE_KI,
                                    updateDt, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE)

        self.__pidRollRate = pidSetIntegralLimit(self.__pidRollRate, PID_ROLL_RATE_INTEGRATION_LIMIT)
        self.__pidPitchRate = pidSetIntegralLimit(self.__pidPitchRate, PID_PITCH_INTEGRATION_LIMIT)
        self.__pidYawRate = pidSetIntegralLimit(self.__pidYawRate, PID_YAW_RATE_INTEGRATION_LIMIT)

        # Attitude pids initialization:
        self.__pidRoll = pidInit(self.__pidRoll, 0, PID_ROLL_KP, PID_ROLL_KD, PID_ROLL_KI, updateDt, ATTITUDE_RATE,
                                ATTITUDE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE)
        self.__pidPitch = pidInit(self.__pidPitch, 0, PID_PITCH_KP, PID_PITCH_KD, PID_PITCH_KI, updateDt, ATTITUDE_RATE,
                                ATTITUDE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE)
        self.__pidYaw = pidInit(self.__pidYaw, 0, PID_YAW_KP, PID_YAW_KD, PID_YAW_KI, updateDt, ATTITUDE_RATE,
                                ATTITUDE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE)

        self.__pidRoll = pidSetIntegralLimit(self.__pidRoll, PID_ROLL_INTEGRATION_LIMIT)
        self.__pidPitch = pidSetIntegralLimit(self.__pidPitch, PID_PITCH_INTEGRATION_LIMIT)
        self.__pidYaw = pidSetIntegralLimit(self.__pidYaw, PID_YAW_INTEGRATION_LIMIT)

    def __attitudeControllerCorrectAttitudePID(self, eulerRollActual, eulerPitchActual, eulerYawActual,
                                               eulerRollDesired, eulerPitchDesired, eulerYawDesired):
        # Roll rate desired:
        self.__pidRoll = pidSetDesired(self.__pidRoll, eulerRollDesired)
        rollRes = pidUpdate(self.__pidRoll, eulerRollActual, True)
        self.__pidRoll = rollRes[0]
        rollRateDesired = rollRes[1]

        # Pitch rate desired:
        self.__pidPitch = pidSetDesired(self.__pidPitch, eulerPitchDesired)
        pitchRes = pidUpdate(self.__pidPitch, eulerPitchActual, True)
        self.__pidPitch = pitchRes[0]
        pitchRateDesired = pitchRes[1]

        # Yaw rate desired:
        yawError = eulerYawDesired - eulerYawActual
        if yawError > 180.0:
            yawError = yawError - 360.0
        elif yawError < -180.0:
            yawError = yawError + 360.0

        self.__pidYaw = pidSetError(self.__pidYaw, yawError)
        yawRes = pidUpdate(self.__pidYaw, eulerYawActual, False)
        self.__pidYaw = yawRes[0]
        yawRateDesired = yawRes[1]

        return (rollRateDesired, pitchRateDesired, yawRateDesired)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                   __A T T I T U D E  R A T E  C O N T R O L L E R
    #
    # ------------------------------------------------------------------------------------------------------------------
    def __attitudeControllerCorrectRatePID(self, rollRateActual, pitchRateActual, yawRateActual,
                                           rollRateDesired, pitchRateDesired, yawRateDesired):
        # Roll output:
        self.__pidRollRate = pidSetDesired(self.__pidRollRate, rollRateDesired)
        rollRateRes = pidUpdate(self.__pidRollRate, rollRateActual, True)
        self.__pidRollRate = rollRateRes[0]
        rollOutput = saturateSignedInt16(rollRateRes[1])

        # Pitch output:
        self.__pidPitchRate = pidSetDesired(self.__pidPitchRate, pitchRateDesired)
        pitchRateRes = pidUpdate(self.__pidPitchRate, pitchRateActual, True)
        self.__pidPitchRate = pitchRateRes[0]
        pitchOutput = saturateSignedInt16(pitchRateRes[1])

        # Yaw output:
        self.__pidYawRate = pidSetDesired(self.__pidYawRate, yawRateDesired)
        yawRateRes = pidUpdate(self.__pidYawRate, yawRateActual, True)
        self.__pidYawRate = yawRateRes[0]
        yawOutput = saturateSignedInt16(yawRateRes[1])

        return (rollOutput, pitchOutput, yawOutput)


    # ==================================================================================================================
    #
    #                       S U P P O R T  M E T H O D S  P O S I T I O N  C O N T R O L L E R S
    #
    # ==================================================================================================================
    def __runPid(self, input, axis, setpoint, dt):
        axis.setpoint = setpoint
        axis.pid = pidSetDesired(axis.pid, axis.setpoint)
        result = pidUpdate(axis.pid, input, True)
        #axis.pid = result[0]
        return result

    def __positionControlResetAllPID(self):
        self.this.pidX.pid = pidReset(self.this.pidX.pid)
        self.this.pidY.pid = pidReset(self.this.pidY.pid)
        self.this.pidZ.pid = pidReset(self.this.pidZ.pid)
        self.this.pidVX.pid = pidReset(self.this.pidVX.pid)
        self.this.pidVY.pid = pidReset(self.this.pidVY.pid)
        self.this.pidVZ.pid = pidReset(self.this.pidVZ.pid)

    # ==================================================================================================================
    #
    #                       S U P P O R T  M E T H O D S  A T T I T U D E  C O N T R O L L E R S
    #
    # ==================================================================================================================
    def __attitudeControllerResetRollAttitudePID(self):
        self.__pidRoll = pidReset(self.__pidRoll)

    def __attitudeControllerResetPitchAttitudePID(self):
        self.__pidPitch = pidReset(self.__pidPitch)

    def __attitudeControllerResetAllPID(self):
        self.__pidRoll = pidReset(self.__pidRoll)
        self.__pidPitch = pidReset(self.__pidPitch)
        self.__pidYaw = pidReset(self.__pidYaw)

        self.__pidRollRate = pidReset(self.__pidRollRate)
        self.__pidPitchRate = pidReset(self.__pidPitchRate)
        self.__pidYawRate = pidReset(self.__pidYawRate)