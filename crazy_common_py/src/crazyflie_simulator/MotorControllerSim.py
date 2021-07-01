# ROS MODULES
import rospy
import time
# MESSAGE
from std_msgs.msg import Float64
from crazyflie_messages.msg import Position, Attitude, CrazyflieState
from geometry_msgs.msg import Wrench

# CUSTOM MODULES
from crazyflie_simulator.FlightControllerSimFirmwr import MAX_THRUST, INT16_MAX
from crazy_common_py.common_functions import RotateVector
from crazy_common_py.dataTypes import Vector3
# OTHER MODULES
from enum import Enum
import math

# SERVICES MESSAGES
from crazyflie_messages.srv import MotorCommand_srv, MotorCommand_srvResponse
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest, BodyRequest, BodyRequestRequest

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

        self.lift_pub = rospy.Publisher('/' + cfName + '/lift_M' + str(motorID), Wrench, queue_size=1)
        self.lift_pub_msg = Wrench()

        self.remove_forces_srv = rospy.ServiceProxy('/gazebo/clear_body_wrenches', BodyRequest)
        self.remove_forces_srv_request = BodyRequestRequest()

        self.actual_state_sub = rospy.Subscriber('/' + cfName + '/state', CrazyflieState, self.__actual_state_callback)
        self.actual_state = CrazyflieState()

        rospy.wait_for_service('/gazebo/clear_body_wrenches')

    def __actual_state_callback(self, msg):
        self.actual_state.orientation.roll = msg.orientation.roll
        self.actual_state.orientation.pitch = msg.orientation.pitch
        self.actual_state.orientation.yaw = msg.orientation.yaw

    def __setVelocitySetpoint(self, thrust):
        # Getting the rotating speed:
        velocity = self.__thrustToVelocity(thrust)

        # Change the sign based on the rotating direction:
        if self.direction == rotatingDirection.CW:
            velocity = velocity * (-1)
        self.__velocitySetpoint.data = velocity

        # Publish the velocity setpoint to the relative controller:
        self.__velocitySetpoint_pub.publish(self.__velocitySetpoint)

    def __setLiftForce(self, thrust):
        # Remove actual force:
        '''self.remove_forces_srv_request.body_name = self.cfName + '::crazyflie_prop_M' + str(self.motorID)
        self.remove_forces_srv(self.remove_forces_srv_request)'''
        # Getting actual state:
        actual_state = self.actual_state

        # Calculating force components:
        force = self.__thrustToLift(thrust)
        res = RotateVector(Vector3(0.0, 0.0, force), Vector3(actual_state.orientation.roll, actual_state.orientation.pitch, actual_state.orientation.yaw))

        # Setting the new force:
        self.lift_pub_msg.force.x = res.x
        self.lift_pub_msg.force.y = res.y
        self.lift_pub_msg.force.z = res.z
        self.lift_pub.publish(self.lift_pub_msg)

    def sendThrustCommand(self, thrust):
        # Setting the velocity setpoint:
        self.__setVelocitySetpoint(thrust)

        # Generating corresponding lift force:
        self.__setLiftForce(thrust)

    def stopMotor(self):
        # Publish the velocity setpoint to the relative controller:
        self.__velocitySetpoint_pub.publish(0)

        # Setting the new force:
        self.lift_pub_msg.force.x = 0.0
        self.lift_pub_msg.force.y = 0.0
        self.lift_pub_msg.force.z = 0.0
        self.lift_pub.publish(self.lift_pub_msg)

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
        return totalLift

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                       __T H R U S T T O V E L O C I T Y
    #
    # For a given thrust command, it converts it into the rotating speed [rad/s] to be sent in Gazebo simulation.
    # ------------------------------------------------------------------------------------------------------------------

    def __thrustToVelocity(self, thrust):
        rotatingSpeed = A1_THRUST_ROTATING_SPEED * thrust + A0_THRUST_ROTATING_SPEED # [rad/s]
        return rotatingSpeed

class MotorControllerSim:
    def __init__(self, cfName):
        # Properties setup:
        self.cfName = cfName
        self.M1 = MotorSim(cfName, 1, rotatingDirection.CCW)
        self.M2 = MotorSim(cfName, 2, rotatingDirection.CW)
        self.M3 = MotorSim(cfName, 3, rotatingDirection.CCW)
        self.M4 = MotorSim(cfName, 4, rotatingDirection.CW)

        self.motor_command_sub = rospy.Subscriber('/' + cfName + '/motor_command', Attitude,
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
        self.M1.sendThrustCommand(thrust_M1)
        self.M2.sendThrustCommand(thrust_M2)
        self.M3.sendThrustCommand(thrust_M3)
        self.M4.sendThrustCommand(thrust_M4)


    def stopMotors(self):
        self.M1.stopMotor()
        self.M2.stopMotor()
        self.M3.stopMotor()
        self.M4.stopMotor()