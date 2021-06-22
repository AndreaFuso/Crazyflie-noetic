# ROS MODULES
import rospy

# CUSTOM MODULES
from crazyflie_simulator.filters import *
from crazyflie_simulator.pid import *
from crazyflie_simulator.stabilizer_types import *

# SERVICE MESSAGES
from crazyflie_messages.srv import DesiredPosition_srv, DesiredPosition_srvResponse, DesiredPosition_srvRequest

# TOPIC MESSAGES
from crazyflie_messages.msg import RollPitchYaw, CrazyflieState

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
        self.this = this_s(pidAxis_s(PidObject(),pidInit_s(25.0, 1.0, 0.0),stab_mode_t.modeDisable),
                           pidAxis_s(PidObject(), pidInit_s(25.0, 1.0, 0.0), stab_mode_t.modeDisable),
                           pidAxis_s(PidObject(), pidInit_s(25.0, 15.0, 0.0), stab_mode_t.modeDisable),
                           pidAxis_s(PidObject(), pidInit_s(2.0, 0.0, 0.0), stab_mode_t.modeDisable),
                           pidAxis_s(PidObject(), pidInit_s(2.0, 0.0, 0.0), stab_mode_t.modeDisable),
                           pidAxis_s(PidObject(), pidInit_s(2.0, 0.5, 0.0), stab_mode_t.modeDisable),
                           36000, 20000)
        self.name = name

        # Initializing the position controller:
        self.positionControllerInit()

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                           S U B S C R I B E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        state_sub = rospy.Subscriber('/' + name + '/state', CrazyflieState, )
        self.actual_state = CrazyflieState()

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                           S E R V I C E S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        self.set_position_target_srv = rospy.Service('/' + name + '/set_target_position', DesiredPosition_srv,
                                                     self.__set_position_target_srv_callback)
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

    def __set_position_target_srv_callback(self, request):
        # Setting up parameters needed by positionController():
        thrust = 0
        attitude = attitude_t()
        setpoint = setpoint_t(
            position=Vector3(request.desired_position.x, request.desired_position.y, request.desired_position.z),
            mode=mode(x=stab_mode_t.modeAbs, y=stab_mode_t.modeAbs, z=stab_mode_t.modeAbs))
        state = state_t(position=Vector3(self.actual_state.position.x, self.actual_state.position.y,
                                  self.actual_state.position.z))
        # Calling positionController():
        result = self.positionController(thrust, attitude, setpoint, state)
        attitude = result[0]
        # Ceating the response:
        response = DesiredPosition_srvResponse()
        response.response_status = True
        response.thrust = result[1]
        response.desired_attitude.roll = attitude.roll
        response.desired_attitude.pitch = attitude.pitch
        response.desired_attitude.yaw = attitude.yaw

        return response

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                           S U B S C R I B E R S  C A L L B A C K S
    #
    # ------------------------------------------------------------------------------------------------------------------
    def __state_sub_callback(self, msg):
        self.actual_state = msg

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
    def positionControllerInit(self):
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

    def positionController(self, thrust, attitude, setpoint, state):
        self.this.pidX.pid.outputLimit = xyVelMax * velMaxOverhead
        self.this.pidY.pid.outputLimit = xyVelMax * velMaxOverhead
        self.this.pidZ.pid.outputLimit = max(zVelMax, 0.5) * velMaxOverhead

        cosyaw = math.cos(state.attitude.yaw * M_PI / 180.0)
        sinyaw = math.sin(state.attitude.yaw * M_PI / 180.0)
        bodyvx = setpoint.velocity.x
        bodyvy = setpoint.velocity.y

        if setpoint.mode.x == stab_mode_t.modeAbs:
            setpoint.velocity.x = self.runPid(state.position.x, self.this.pidX, setpoint.position.x, DT)
        elif setpoint.velocity_body:
            setpoint.velocity.x = bodyvx * cosyaw - bodyvy * sinyaw

        if setpoint.mode.y == stab_mode_t.modeAbs:
            setpoint.velocity.y = self.runPid(state.position.y, self.this.pidY, setpoint.position.y, DT)
        elif setpoint.velocity_body:
            setpoint.velocity.y = bodyvy * cosyaw + bodyvx * sinyaw

        if setpoint.mode.z == stab_mode_t.modeAbs:
            setpoint.velocity.z = self.runPid(state.position.z, self.this.pidZ, setpoint.position.z, DT)

        return self.velocityController(thrust, attitude, setpoint, state)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                           __V E L O C I T Y  C O N T R O L L E R
    #
    # ------------------------------------------------------------------------------------------------------------------
    def velocityController(self, thrust, attitude, setpoint, state):
        self.this.pidVX.pid.outputLimit = rpLimit * rpLimitOverhead
        self.this.pidVY.pid.outputLimit = rpLimit * rpLimitOverhead
        self.this.pidVZ.pid.outputLimit = 65535 / 2 / thrustScale

        rollRaw = self.runPid(state.velocity.x, self.this.pidVX, setpoint.velocity.x, DT)
        pitchRaw = self.runPid(state.velocity.y, self.this.pidVY, setpoint.velocity.y, DT)
        yawRad = state.attitude.yaw * M_PI / 180

        attitude.pitch = - (rollRaw * math.cos(yawRad)) - (pitchRaw * math.sin(yawRad))
        attitude.roll = - (pitchRaw * math.cos(yawRad)) + (rollRaw * math.sin(yawRad))

        attitude.roll = constrain(attitude.roll, -rpLimit, rpLimit)
        attitude.pitch = constrain(attitude.pitch, -rpLimit, rpLimit)

        thrustRaw = self.runPid(state.velocity.z, self.this.pidVZ, setpoint.velocity.z, DT)
        thrust = thrustRaw * thrustScale + self.this.thrustBase

        if thrust < self.this.thrustMin:
            thrust = self.this.thrustMin

        return (attitude, thrust)

    # ==================================================================================================================
    #
    #                       S U P P O R T  M E T H O D S  P O S I T I O N  C O N T R O L L E R S
    #
    # ==================================================================================================================
    def runPid(self, input, axis, setpoint, dt):
        axis.setpoint = setpoint
        axis.pid = pidSetDesired(axis.pid, axis.setpoint)
        result = pidUpdate(axis.pid, input, True)
        axis.pid = result[0]
        return result[1]

    def positionControlResetAllPID(self):
        self.this.pidX.pid = pidReset(self.this.pidX.pid)
        self.this.pidY.pid = pidReset(self.this.pidY.pid)
        self.this.pidZ.pid = pidReset(self.this.pidZ.pid)
        self.this.pidVX.pid = pidReset(self.this.pidVX.pid)
        self.this.pidVY.pid = pidReset(self.this.pidVY.pid)
        self.this.pidVZ.pid = pidReset(self.this.pidVZ.pid)
