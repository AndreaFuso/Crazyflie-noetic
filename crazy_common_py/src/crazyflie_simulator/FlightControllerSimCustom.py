# ROS MODULES
import rospy

# GENERAL MODULES
import math

# CUSTOM MODULES
from crazy_common_py.controllers import PidController, WindupType, WindupInfo
from crazy_common_py.constants import *
from crazy_common_py.common_functions import rad2deg, constrain, isSameVector
from crazy_common_py.dataTypes import Vector3

# TOPIC MESSAGES
from crazyflie_messages.msg import CrazyflieState, Attitude, Position, RollPitchYaw
from std_msgs.msg import Empty

# NUMBERS
INT16_MAX = 32767

THRUST_BASE = 38180
thrustScale = 1000.0

DEFAULT_WINDUP_TYPE = WindupType.Exclusion

# CONSTANTS
MAX_VELOCITY_X = 1.0    # [m/s]
MAX_VELOCITY_Y = 1.0    # [m/s]
MAX_VELOCITY_Z = 1.0    # [m/s]

MAX_THRUST = 65535
MIN_THRUST = 20000

MAX_PITCH = 20  # [deg]
MAX_ROLL = 20   # [deg]

'''DELTA_THRUST = min(MAX_THRUST - THRUST_BASE, THRUST_BASE - MIN_THRUST)
MAX_ROLL_OUTPUT = 1.0 * DELTA_THRUST  #5000
MAX_PITCH_OUTPUT = 1.0 * DELTA_THRUST #5000
MAX_YAW_OUTPUT = 1.0 * DELTA_THRUST #6000'''

MAX_ROLL_OUTPUT = 7000 #5000
MAX_PITCH_OUTPUT = 7000 #5000
MAX_YAW_OUTPUT = 7000 #6000



class FlightControllerCustom:
    # ==================================================================================================================
    #
    #                                               C O N S T R U C T O R
    #
    # This class replicates the internal flight controller of the Crazyflie, even if all the four pids running in the
    # real cf are implemented, just three of them are used (position, velocity and attitude).
    # INPUTS:
    #   1) cfName -> name of the crazyflie in the simulation;
    # ==================================================================================================================
    def __init__(self, cfName):
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                           P R O P E R T I E S  I N I T I A L I Z A T I O N
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Name of the virtual Crazyflie:
        self.cfName = cfName

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                    S U B S C R I B E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        self.pace_100Hz_sub = rospy.Subscriber('/pace_100Hz', Empty, self.__pace_100Hz_callback)
        self.pace_500Hz_sub = rospy.Subscriber('/pace_500Hz', Empty, self.__pace_500Hz_callback)

        # Subscriber to get the actual state of the drone in the simulation:
        state_sub = rospy.Subscriber('/' + cfName + '/state', CrazyflieState, self.__state_sub_callback)
        self.actual_state = CrazyflieState()

        # Subscriber looking for a point to go:
        self.desired_position_sub = rospy.Subscriber('/' + cfName + '/set_destination_position', Position,
                                                     self.__desired_position_sub_callback)

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                      P U B L I S H E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # self.desired_attitude_pub = rospy.Publisher('/' + name + '/set_desired_attitude', Attitude)

        self.motor_command_pub = rospy.Publisher('/' + cfName + '/set_desired_motor_command', Attitude, queue_size=1)
        self.desired_motor_command = Attitude()

        self.previous_desired_position = Vector3(0.0001, 0.0001, 0.0001)

        self.__init_position_controller()
        self.__init_velocity_controller()
        self.__init_attitude_controller()
        self.__init_attitude_rate_controller()

        self.OK = False
        self.OK500 = False

    # ==================================================================================================================
    #
    #                                               C A L L B A C K S
    #
    # ==================================================================================================================
    def __pace_100Hz_callback(self, msg):
        pass

    def __pace_500Hz_callback(self, msg):
        if self.OK500:
            # Saving actual state:
            actual_state = self.actual_state

            # Desired thrust:
            desired_thrust = self.desired_thrust
            # Desired position velocity:
            desired_position = self.desired_position
            desired_velocity = self.desired_velocity
            # Saving actual desired attitude:
            desired_attitude = self.desired_attitude
            desired_attitude.z = self.desired_yaw

            '''print('\nDESIRED POSITION: ', desired_position.x, '; ', desired_position.y, '; ', desired_position.z,
                  '\nACTUAL POSITION: ', actual_state.position.x, '; ', actual_state.position.y, '; ',
                  actual_state.position.z)

            print('DESIRED VELOCITY: ', desired_velocity.x, '; ', desired_velocity.y, '; ', desired_velocity.z,
                  '\nACTUAL VELOCITY: ', actual_state.velocity.x, '; ', actual_state.velocity.y, '; ',
                  actual_state.velocity.z)


            print('DESIRED ATTITUDE: ', desired_attitude.x, '; ', desired_attitude.y, '; ', desired_attitude.z,
                  '\nACTUAL ATTITUDE: ', actual_state.orientation.roll, '; ', actual_state.orientation.pitch, '; ', actual_state.orientation.yaw)'''

            # Calling attitude controller:
            desired_rpy_command = self.__attitudeController(desired_attitude, actual_state)
            #print('DESIRED RPY COMMAND: ', desired_rpy_command.x, '; ', desired_rpy_command.y, ';', desired_rpy_command.z)

            motor_commands = self.__motor_simulator(desired_rpy_command, desired_thrust)


            #print('MOTOR COMMANDS: ', motor_commands[0], '; ', motor_commands[1], '; ', motor_commands[2], '; ', motor_commands[3], '\n')
            # Setting up messaghe to be published:
            self.desired_motor_command.desired_attitude.roll = desired_rpy_command.x
            self.desired_motor_command.desired_attitude.pitch = desired_rpy_command.y
            self.desired_motor_command.desired_attitude.yaw = desired_rpy_command.z
            self.desired_motor_command.desired_thrust = self.desired_thrust

            # Publishing motor command:
            self.motor_command_pub.publish(self.desired_motor_command)

    def __motor_simulator(self, desired_attitude, desired_thrust):
        # Extracting request info:
        roll = desired_attitude.x / 2.0
        pitch = desired_attitude.y / 2.0
        yaw = desired_attitude.z
        thrust = desired_thrust

        # Calculating the thrust for each motor:
        '''thrust_M1 = constrain(thrust - roll + pitch + yaw, 0, MAX_THRUST)
        thrust_M2 = constrain(thrust - roll - pitch - yaw, 0, MAX_THRUST)
        thrust_M3 = constrain(thrust + roll - pitch + yaw, 0, MAX_THRUST)
        thrust_M4 = constrain(thrust + roll + pitch - yaw, 0, MAX_THRUST)'''

        thrust_M1 = constrain(thrust - roll - pitch - yaw, 0, MAX_THRUST)
        thrust_M2 = constrain(thrust - roll + pitch + yaw, 0, MAX_THRUST)
        thrust_M3 = constrain(thrust + roll + pitch - yaw, 0, MAX_THRUST)
        thrust_M4 = constrain(thrust + roll - pitch + yaw, 0, MAX_THRUST)

        return  (thrust_M1, thrust_M2, thrust_M3, thrust_M4)

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
        # Saving actual state and desired position:
        actual_state = self.actual_state

        desired_position = Vector3(msg.desired_position.x, msg.desired_position.y, msg.desired_position.z)

        self.desired_position = desired_position
        self.desired_yaw = msg.desired_yaw

        # Check if there's a new desired position:
        if not isSameVector(self.previous_desired_position, desired_position):
            # Updating previous desired position:
            self.previous_desired_position = desired_position

            # Resetting pids:
            self.PID_position_x.reset()
            self.PID_position_y.reset()
            self.PID_position_z.reset()

            self.PID_velocity_x.reset()
            self.PID_velocity_y.reset()
            self.PID_velocity_z.reset()

            self.PID_attitude_roll.reset()
            self.PID_attitude_pitch.reset()
            self.PID_attitude_yaw.reset()

            self.PID_attitude_roll_rate.reset()
            self.PID_attitude_pitch_rate.reset()
            self.PID_attitude_yaw_rate.reset()

            self.OK = True
            self.OK500 = False
            #print('\n\n\n\n\n\n\n\n NEW POSITION!!!\n\n\n\n\n\n\n\n')

        # Calling positionController to get desired velocities:
        desired_velocity = self.__positionController(desired_position, actual_state)

        self.desired_velocity = desired_velocity

        # Calling velocityController to get desired attitude:
        result = self.__velocityController(desired_velocity, actual_state)
        if self.OK:
            self.desired_attitude = result[0]
            self.desired_thrust = result[1]
            self.OK500 = True
        #print('DESIRED ATTITUDE: ', self.desired_attitude.x, '; ', self.desired_attitude.y, '; ', self.desired_attitude.z)
        #print('DESIRED THRUST: ', self.desired_thrust, '/', MAX_THRUST)



    # ==================================================================================================================
    #
    #                                               C O N T R O L L E R S
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                        P O S I T I O N  C O N T R O L L E R
    #
    # ------------------------------------------------------------------------------------------------------------------
    def __init_position_controller(self):
        windup_type = DEFAULT_WINDUP_TYPE
        windup_info = WindupInfo()
        dt = 1 / 100
        self.PID_position_x = PidController(Kp=PID_POSITION_X_KP, Ki=PID_POSITION_X_KI, Kd=PID_POSITION_X_KD,
                                            dt=dt, windupType=windup_type, integralLimit=DEFAULT_PID_INTEGRATION_LIMIT,
                                            windup_info=WindupInfo(- MAX_VELOCITY_X, MAX_VELOCITY_X))
        self.PID_position_y = PidController(Kp=PID_POSITION_Y_KP, Ki=PID_POSITION_Y_KI, Kd=PID_POSITION_Y_KD,
                                            dt=dt, windupType=windup_type, integralLimit=DEFAULT_PID_INTEGRATION_LIMIT,
                                            windup_info=WindupInfo(- MAX_VELOCITY_Y, MAX_VELOCITY_Y))
        self.PID_position_z = PidController(Kp=PID_POSITION_Z_KP, Ki=PID_POSITION_Z_KI, Kd=PID_POSITION_Z_KD,
                                            dt=dt, windupType=windup_type, integralLimit=DEFAULT_PID_INTEGRATION_LIMIT,
                                            windup_info=WindupInfo(- MAX_VELOCITY_Z, MAX_VELOCITY_Z))

    def __positionController(self, desiredPosition=Vector3(), actualState=CrazyflieState()):
        # Output:
        desired_velocity = Vector3()

        # Calculating desired velocities through pid:
        desired_velocity.x = self.PID_position_x.updatePID(actualState.position.x, desiredPosition.x)
        desired_velocity.y = self.PID_position_y.updatePID(actualState.position.y, desiredPosition.y)
        desired_velocity.z = self.PID_position_z.updatePID(actualState.position.z, desiredPosition.z)

        # Limitating values (to be sure):
        desired_velocity.x = constrain(desired_velocity.x, - MAX_VELOCITY_X, MAX_VELOCITY_X)
        desired_velocity.y = constrain(desired_velocity.y, - MAX_VELOCITY_Y, MAX_VELOCITY_Y)
        desired_velocity.z = constrain(desired_velocity.z, - MAX_VELOCITY_Z, MAX_VELOCITY_Z)

        return desired_velocity

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                        V E L O C I T Y  C O N T R O L L E R
    #
    # ------------------------------------------------------------------------------------------------------------------
    def __init_velocity_controller(self):
        windup_type = DEFAULT_WINDUP_TYPE
        windup_info = WindupInfo()
        dt = 1 / 100
        self.PID_velocity_x = PidController(Kp=PID_VELOCITY_X_KP, Ki=PID_VELOCITY_X_KI, Kd=PID_VELOCITY_X_KD,
                                            dt=dt, windupType=windup_type, integralLimit=DEFAULT_PID_INTEGRATION_LIMIT,
                                            windup_info=WindupInfo(- MAX_ROLL, MAX_ROLL))
        self.PID_velocity_y = PidController(Kp=PID_VELOCITY_Y_KP, Ki=PID_VELOCITY_Y_KI, Kd=PID_VELOCITY_Y_KD,
                                            dt=dt, windupType=windup_type, integralLimit=DEFAULT_PID_INTEGRATION_LIMIT,
                                            windup_info=WindupInfo(- MAX_PITCH, MAX_PITCH))
        self.PID_velocity_z = PidController(Kp=PID_VELOCITY_Z_KP, Ki=PID_VELOCITY_Z_KI, Kd=PID_VELOCITY_Z_KD,
                                            dt=dt, windupType=windup_type, integralLimit=DEFAULT_PID_INTEGRATION_LIMIT,
                                            windup_info=WindupInfo(MIN_THRUST, MAX_THRUST))

    def __velocityController(self, desiredVelocity=Vector3(), actualState=CrazyflieState()):
        # Output [roll, pitch, yaw]:
        desired_attitude = Vector3()

        # Calculating the desired attitude:
        deltaVX = self.PID_velocity_x.updatePID(actualState.velocity.x, desiredVelocity.x)
        deltaVY = self.PID_velocity_y.updatePID(actualState.velocity.y, desiredVelocity.y)

        yawRad = actualState.orientation.yaw * M_PI / 180.0

        # MODIFICA

        '''desired_attitude.x = - (deltaVY * math.cos(yawRad)) + (deltaVX * math.sin(yawRad))
        desired_attitude.y = - (deltaVX * math.cos(yawRad)) - (deltaVY * math.sin(yawRad))'''

        '''desired_attitude.x =  - (deltaVY * math.cos(yawRad)) + (deltaVX * math.sin(yawRad))
        desired_attitude.y =  (deltaVX * math.cos(yawRad)) + (deltaVY * math.sin(yawRad))'''

        desired_attitude.x = deltaVX * math.sin(yawRad) - deltaVY * math.cos(yawRad)
        desired_attitude.y = deltaVX * math.cos(yawRad) + deltaVY * math.sin(yawRad)

        # Calculating desired thrust:
        rawThrust = self.PID_velocity_z.updatePID(actualState.velocity.z, desiredVelocity.z)

        desired_thrust = rawThrust * thrustScale + THRUST_BASE

        # Clambing to be sure:
        desired_attitude.x = constrain(desired_attitude.x, - MAX_ROLL, MAX_ROLL)
        desired_attitude.y = constrain(desired_attitude.y, -MAX_PITCH, MAX_PITCH)
        desired_thrust = constrain(desired_thrust, MIN_THRUST, MAX_THRUST)

        return (desired_attitude, desired_thrust)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                        A T T I T U D E  C O N T R O L L E R
    #
    # ------------------------------------------------------------------------------------------------------------------
    def __init_attitude_controller(self):
        windup_type = DEFAULT_WINDUP_TYPE
        windup_type = WindupType.Clamped
        windup_info = WindupInfo()
        dt = 1 / 500
        self.PID_attitude_roll = PidController(Kp=ROLL_KP, Ki=ROLL_KI, Kd=ROLL_KD,
                                               dt=dt, windupType=windup_type, integralLimit=PID_ROLL_INTEGRATION_LIMIT,
                                               windup_info=WindupInfo(- MAX_ROLL_OUTPUT, MAX_ROLL_OUTPUT))
        self.PID_attitude_pitch = PidController(Kp=PITCH_KP, Ki=PITCH_KI, Kd=PITCH_KD,
                                                dt=dt, windupType=windup_type, integralLimit=PID_PITCH_INTEGRATION_LIMIT,
                                                windup_info=WindupInfo(- MAX_PITCH_OUTPUT, MAX_PITCH_OUTPUT))
        self.PID_attitude_yaw = PidController(Kp=YAW_KP, Ki=YAW_KI, Kd=YAW_KD,
                                              dt=dt, windupType=windup_type, integralLimit=PID_YAW_INTEGRATION_LIMIT,
                                              windup_info=WindupInfo(- MAX_YAW_OUTPUT, MAX_YAW_OUTPUT))

    def __attitudeController(self, desiredAttitude=Vector3(), actualState=CrazyflieState()):
        # Output [rollCmd, pitchCmd, yawCmd]:
        desired_output_command = Vector3()

        # Calculating desired output command (roll and pitch):
        desired_output_command.x = self.PID_attitude_roll.updatePID(actualState.orientation.roll, desiredAttitude.x)
        desired_output_command.y = self.PID_attitude_pitch.updatePID(actualState.orientation.pitch, desiredAttitude.y)

        # Calculating desired output command for yaw:
        yawError = desiredAttitude.z - actualState.orientation.yaw
        if yawError > 180.0:
            yawError = yawError - 360.0
        elif yawError < -180.0:
            yawError = yawError + 360.0

        self.PID_attitude_yaw.error = yawError
        desired_output_command.z = self.PID_attitude_yaw.updatePID(actualState.orientation.yaw, desiredAttitude.z, False)

        # Clamping to be sure:
        desired_output_command.x = constrain(desired_output_command.x, -MAX_ROLL_OUTPUT, MAX_ROLL_OUTPUT)
        desired_output_command.y = constrain(desired_output_command.y, -MAX_PITCH_OUTPUT, MAX_PITCH_OUTPUT)
        desired_output_command.z = constrain(desired_output_command.z, -MAX_YAW_OUTPUT, MAX_YAW_OUTPUT)

        return desired_output_command

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                        A T T I T U D E  R A T E  C O N T R O L L E R
    #
    # ------------------------------------------------------------------------------------------------------------------
    def __init_attitude_rate_controller(self):
        windup_type = DEFAULT_WINDUP_TYPE
        windup_info = WindupInfo()
        dt = 1 / 500
        self.PID_attitude_roll_rate = PidController(Kp=PID_ROLL_RATE_KP, Ki=PID_ROLL_RATE_KI, Kd=PID_ROLL_RATE_KD,
                                               dt=dt, windupType=windup_type,
                                               integralLimit=PID_ROLL_RATE_INTEGRATION_LIMIT,
                                               windup_info=WindupInfo(- INT16_MAX, INT16_MAX))
        self.PID_attitude_pitch_rate = PidController(Kp=PID_PITCH_RATE_KP, Ki=PID_PITCH_RATE_KI, Kd=PID_PITCH_RATE_KD,
                                                dt=dt, windupType=windup_type,
                                                integralLimit=PID_PITCH_RATE_INTEGRATION_LIMIT,
                                                windup_info=WindupInfo(- INT16_MAX, INT16_MAX))
        self.PID_attitude_yaw_rate = PidController(Kp=PID_YAW_RATE_KP, Ki=PID_YAW_RATE_KI, Kd=PID_YAW_RATE_KD,
                                              dt=dt, windupType=windup_type,
                                              integralLimit=PID_YAW_RATE_INTEGRATION_LIMIT,
                                              windup_info=WindupInfo(- INT16_MAX, INT16_MAX))

    def __attitudeRateController(self, desiredAttitudeRate=Vector3(), actualState=CrazyflieState()):
        # Output:
        rpy_motor_command = Vector3()

        # Calculating roll pitch yaw motor commands:
        rpy_motor_command.x = self.PID_attitude_roll_rate.updatePID(desiredAttitudeRate.x, actualState.rotating_speed.x)
        rpy_motor_command.y = self.PID_attitude_pitch_rate.updatePID(desiredAttitudeRate.y, actualState.rotating_speed.z)
        rpy_motor_command.z = self.PID_attitude_yaw_rate.updatePID(desiredAttitudeRate.z, actualState.rotating_speed.z)

        # Clamping to be sure:
        rpy_motor_command.x = constrain(rpy_motor_command.x, -INT16_MAX, INT16_MAX)
        rpy_motor_command.y = constrain(rpy_motor_command.y, -INT16_MAX, INT16_MAX)
        rpy_motor_command.z = constrain(rpy_motor_command.z, -INT16_MAX, INT16_MAX)

        return rpy_motor_command