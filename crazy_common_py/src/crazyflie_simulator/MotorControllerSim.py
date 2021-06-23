# ROS MODULES
import rospy

# MESSAGE
from std_msgs.msg import Float64
from crazyflie_messages.msg import Position, Attitude

# CUSTOM MODULES
from crazyflie_simulator.FlightControllerSim import MAX_THRUST, INT16_MAX

# OTHER MODULES
from enum import Enum

# SERVICES MESSAGES
from crazyflie_messages.srv import MotorCommand_srv, MotorCommand_srvResponse

# CONSTANTS
# Polyfit thrust - lift (Crazyflie Modelling Paper)
A0_THRUST_LIFT = 5.484560e-4
A1_THRUST_LIFT = 1.032633e-6
A2_THRUST_LIFT = 2.130295e-11

# Polyfit thrust - rotating speed (Crazyflie Modelling Paper)
A0_THRUST_ROTATING_SPEED = 380.8359
A1_THRUST_ROTATING_SPEED = 0.04076521

def limitThrust(thrust):
    if thrust > MAX_THRUST:
        thrust = MAX_THRUST
    elif thrust < 0:
        thrust = 0
    return thrust

class rotatingDirection(Enum):
    CW = 1
    CCW = -1

class MotorSim:
    # ==================================================================================================================
    #
    #                                               C O N S T R U C T O R
    #
    # This class completely handle one virtual Crazyflie.
    # INPUTS:
    #   1) cfName -> name of the crazyflie in the simulation;
    #   2) motorID -> number of the motor;
    #   3) direction -> rotatingDirection enum, to understand if clockwise or counterclockwise.
    # ==================================================================================================================
    def __init__(self, cfName, motorID, direction):
        # Properties setup:
        self.cfName = cfName
        self.motorID = motorID
        self.direction = direction

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                           S U B S C R I B E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                           P U B L I S H E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        self.__velocitySetpoint_pub = rospy.Publisher('/' + cfName + '/crazyflie_M' + str(motorID) +
                                                      '_joint_velocity_controller/command', Float64, queue_size=1)
        self.__velocitySetpoint = Float64()





    def __setVelocitySetpoint(self, thrust):
        # Getting the rotating speed:
        velocity = self.__thrustToVelocity(thrust)

        # Change the sign based on the rotating direction:
        if self.direction == rotatingDirection.CCW:
            velocity = velocity * (-1)
        self.__velocitySetpoint.data = velocity

        # Publish the velocity setpoint to the relative controller:
        self.__velocitySetpoint_pub.publish(self.__velocitySetpoint)

    def __setLiftForce(self, thrust, duration):
        pass

    def sendThrustCommand(self, thrust, duration):
        # Setting the velocity setpoint:
        self.__setVelocitySetpoint(thrust)

        # Generating corresponding lift force:
        self.__setLiftForce(thrust, duration)

    # ==================================================================================================================
    #
    #                                               M A P P I N G  M E T H O D S
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                           __T H R U S T T O L I F T
    #
    # For a given thrust command, it converts it into a force [N] (lift force) to be sent in Gazebo simulation.
    # ------------------------------------------------------------------------------------------------------------------

    def __thrustToLift(self, thrust):
        totalLift = A2_THRUST_LIFT * (thrust ** 2) + A1_THRUST_LIFT * thrust + A0_THRUST_LIFT   # [N]
        return totalLift / 4

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                       __T H R U S T T O V E L O C I T Y
    #
    # For a given thrust command, it converts it into the rotating speed [rad/s] to be sent in Gazebo simulation.
    # ------------------------------------------------------------------------------------------------------------------

    def __thrustToVelocity(self, thrust):
        rotatinSpeed = A1_THRUST_ROTATING_SPEED * thrust + A0_THRUST_ROTATING_SPEED # [rad/s]
        return rotatinSpeed

class MotorControllerSim:
    def __init__(self, cfName):
        # Properties setup:
        self.cfName = cfName
        self.M1 = MotorSim(cfName, 1, rotatingDirection.CCW)
        self.M2 = MotorSim(cfName, 2, rotatingDirection.CW)
        self.M3 = MotorSim(cfName, 3, rotatingDirection.CCW)
        self.M4 = MotorSim(cfName, 4, rotatingDirection.CW)

        self.motor_command_srv = rospy.Service('/' + cfName + '/motor_command', MotorCommand_srv,
                                               self.__motor_command_srv_callback)
        self.motor_command_sub = rospy.Subscriber('/' + cfName + '/set_desired_motor_command', Attitude,
                                                  self.__motor_command_sub_callback)

    def __motor_command_sub_callback(self, msg):
        # Extracting request info:
        roll = msg.desired_attitude.roll / 2.0
        pitch = msg.desired_attitude.pitch / 2.0
        yaw = msg.desired_attitude.yaw
        thrust = msg.desired_thrust

        # Calculating the thrust for each motor:
        thrust_M1 = limitThrust(thrust - roll + pitch + yaw)
        thrust_M2 = limitThrust(thrust - roll - pitch - yaw)
        thrust_M3 = limitThrust(thrust + roll - pitch + yaw)
        thrust_M4 = limitThrust(thrust + roll + pitch - yaw)

        # Sending the thrust command:
        self.M1.sendThrustCommand(thrust_M1, 1)
        self.M2.sendThrustCommand(thrust_M2, 1)
        self.M3.sendThrustCommand(thrust_M3, 1)
        self.M4.sendThrustCommand(thrust_M4, 1)

    def __motor_command_srv_callback(self, request):
        # Extracting request info:
        roll = request.rpy_output.roll / 2.0
        pitch = request.rpy_output.pitch / 2.0
        yaw = request.rpy_output.yaw
        thrust = request.thrust

        # Calculating the thrust for each motor:
        thrust_M1 = limitThrust(thrust - roll + pitch + yaw)
        thrust_M2 = limitThrust(thrust - roll - pitch - yaw)
        thrust_M3 = limitThrust(thrust + roll - pitch + yaw)
        thrust_M4 = limitThrust(thrust + roll + pitch - yaw)

        # Sending the thrust command:
        self.M1.sendThrustCommand(thrust_M1, 1)
        self.M2.sendThrustCommand(thrust_M2, 1)
        self.M3.sendThrustCommand(thrust_M3, 1)
        self.M4.sendThrustCommand(thrust_M4, 1)

        # Response formulation:
        response = MotorCommand_srvResponse()
        response.result = True

        return response

    def stopMotors(self):
        self.M1.sendThrustCommand(0, 1)
        self.M2.sendThrustCommand(0, 1)
        self.M3.sendThrustCommand(0, 1)
        self.M4.sendThrustCommand(0, 1)