# GENERAL MODULES
from enum import Enum
# CUSTOM MODULES
from crazy_common_py.dataTypes import Vector3
# CONSTATNTS
RATE_1000_HZ = 1000
RATE_500_HZ = 500
RATE_250_HZ = 250
RATE_100_HZ = 100
RATE_50_HZ = 50
RATE_25_HZ = 25

RATE_MAIN_LOOP = RATE_1000_HZ
ATTITUDE_RATE = RATE_500_HZ
POSITION_RATE = RATE_100_HZ


class stab_mode_t(Enum):
    modeDisable = 0
    modeAbs = 1
    modeVelocity = 2

class mode:
    def __init__(self, x=stab_mode_t.modeVelocity, y=stab_mode_t.modeVelocity, z=stab_mode_t.modeVelocity,
                 roll=stab_mode_t.modeVelocity, pitch=stab_mode_t.modeVelocity, yaw=stab_mode_t.modeVelocity,
                 quat=stab_mode_t.modeVelocity):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.quat = quat

class attitude_t:
    def __init__(self, timestamp=0, roll=0, pitch=0, yaw=0):
        self.timestamp = timestamp
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
class quaternion_t:
    def __init__(self, timestamp=0, x=0, y=0, z=0, w=0):
        self.timestamp = timestamp
        self.x = x
        self.y = y
        self.z = z
        self.w = w

class setpoint_t:
    def __init__(self, timestamp=0, attitude=attitude_t(), attitudeRate=attitude_t(), attitudeQuaternion=quaternion_t(),
                 thrust=0, position=Vector3(), velocity=Vector3(), acceleration=Vector3(), velocity_body=False, mode=mode()):
        self.timestamp = timestamp
        self.attitude = attitude
        self.attitudeRate = attitudeRate
        self.attitudeQuaternion = attitudeQuaternion
        self.thrust = thrust
        self.position = position
        self.velocity = velocity
        self.acceleration = acceleration
        self.velocity_body = velocity_body
        self.mode = mode

class state_t:
    def __init__(self, attitude=attitude_t(), attitudeQuaternion=quaternion_t(), position=Vector3(), velocity=Vector3(),
                 acc=Vector3()):
        self.attitude = attitude
        self.attitudeQuaternion = attitudeQuaternion
        self.position = position
        self.velocity = velocity
        self.acc = acc
