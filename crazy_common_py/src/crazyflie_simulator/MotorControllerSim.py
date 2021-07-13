# ROS MODULES
import rospy
import time
# MESSAGE
from std_msgs.msg import Float64
from crazyflie_messages.msg import Position, Attitude, CrazyflieState
from geometry_msgs.msg import Wrench

# CUSTOM MODULES
from crazyflie_simulator.FlightControllerSimFirmwr import MAX_THRUST, INT16_MAX
from crazy_common_py.common_functions import RotateVector, constrain
from crazy_common_py.dataTypes import Vector3, rotatingDirection
from crazy_common_py.constants import *

# OTHER MODULES
from enum import Enum
import math

# SERVICES MESSAGES
from crazyflie_messages.srv import MotorCommand_srv, MotorCommand_srvResponse
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest, BodyRequest, BodyRequestRequest


class MotorSim:
    # ==================================================================================================================
    #
    #                                               C O N S T R U C T O R
    #
    # This class completely handles one virtual motor of a Crazyflie.
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
        #                                   S U B S C R I B E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Subscriber to receive informations about the state of the virtual Crazyflie:
        self.actual_state_sub = rospy.Subscriber('/' + cfName + '/state', CrazyflieState, self.__actual_state_callback)
        self.actual_state = CrazyflieState()

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       P U B L I S H E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Publisher to sends velocity setpoint to Gazebo pid controller:
        self.__velocitySetpoint_pub = rospy.Publisher('/' + cfName + '/crazyflie_M' + str(motorID) +
                                                      '_joint_velocity_controller/command', Float64, queue_size=1)
        self.__velocitySetpoint = Float64()

        # Publisher to send the force&torque state to the Gazebo simulation:
        self.lift_drag_pub = rospy.Publisher('/' + cfName + '/lift_M' + str(motorID), Wrench, queue_size=1)
        self.lift_drag_pub_msg = Wrench()


        #rospy.wait_for_service('/gazebo/clear_body_wrenches')

    # ==================================================================================================================
    #
    #                                       C A L L B A C K S  M E T H O D S
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                   __A C T U A L S T A T E C A L L B A C K
    #
    # Everytime the actual state is received, this callback saves the informations whitin an internal variable.
    # ------------------------------------------------------------------------------------------------------------------

    def __actual_state_callback(self, msg):
        self.actual_state.orientation.roll = msg.orientation.roll
        self.actual_state.orientation.pitch = msg.orientation.pitch
        self.actual_state.orientation.yaw = msg.orientation.yaw

    # ==================================================================================================================
    #
    #                                       S I M U L A T I O N  M E T H O D S
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                   __S E T V E L O C I T Y S E T P O I N T
    #
    # This method publishes the reference velocity to the corresponding propeller in Gazebo simulation.
    # ------------------------------------------------------------------------------------------------------------------

    def __setVelocitySetpoint(self, input):
        # Getting the rotating speed:
        velocity = self.__inputCommandToVelocity(input)

        # Change the sign based on the rotating direction:
        if self.direction == rotatingDirection.CW:
            velocity = velocity * (-1.0)
        self.__velocitySetpoint.data = velocity

        # Publish the velocity setpoint to the relative controller:
        self.__velocitySetpoint_pub.publish(self.__velocitySetpoint)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                   __S E T F O R C E T O R Q U E
    #
    # This method sets up the force & torque state we have around the propeller:
    #   - lift force;
    #   - torque due to the drag force;
    # ------------------------------------------------------------------------------------------------------------------

    def __setForceTorque(self, input):
        # Getting actual state:
        actual_state = self.actual_state

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                           F O R C E S
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        # Calculating lift force components:
        liftForce = self.__inputCommandToThrust(input)
        resLiftForce = RotateVector(Vector3(0.0, 0.0, liftForce),
                                Vector3(actual_state.orientation.roll,
                                        actual_state.orientation.pitch,
                                        actual_state.orientation.yaw))

        # Setting the new force:
        self.lift_drag_pub_msg.force.x = resLiftForce.x
        self.lift_drag_pub_msg.force.y = resLiftForce.y
        self.lift_drag_pub_msg.force.z = resLiftForce.z

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                           T O R Q U E S
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        # Calculating drag torque components:
        dragTorque = self.__inputCommandToTorque(input)
        if self.direction == rotatingDirection.CCW:
            dragTorque = (-1.0) * dragTorque

        resDragTorque = RotateVector(Vector3(0.0, 0.0, dragTorque),
                                 Vector3(actual_state.orientation.roll,
                                         actual_state.orientation.pitch,
                                         actual_state.orientation.yaw))
        # Setting the new torque:
        self.lift_drag_pub_msg.torque.x = resDragTorque.x
        self.lift_drag_pub_msg.torque.y = resDragTorque.y
        self.lift_drag_pub_msg.torque.z = resDragTorque.z

        self.lift_drag_pub.publish(self.lift_drag_pub_msg)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                       S E N D I N P U T C O M M A N D
    #
    # This method performs all the operations in the simulation for a certain input command (it sets the rotating
    # target velocity for propellers, sets up the force and torque).
    # ------------------------------------------------------------------------------------------------------------------

    def sendInputCommand(self, input):
        # Setting the velocity setpoint:
        self.__setVelocitySetpoint(input)

        # Generating corresponding resulting force and torque:
        self.__setForceTorque(input)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                               S T O P M O T O R
    #
    # This method stops the motor.
    # ------------------------------------------------------------------------------------------------------------------

    def stopMotor(self):
        # Publish the velocity setpoint to the relative controller:
        self.__velocitySetpoint_pub.publish(0)

        # Setting up a null force:
        self.lift_drag_pub_msg.force.x = 0.0
        self.lift_drag_pub_msg.force.y = 0.0
        self.lift_drag_pub_msg.force.z = 0.0

        # Setting up a null torque:
        self.lift_drag_pub_msg.torque.x = 0.0
        self.lift_drag_pub_msg.torque.y = 0.0
        self.lift_drag_pub_msg.torque.z = 0.0

        self.lift_drag_pub.publish(self.lift_drag_pub_msg)

    # ==================================================================================================================
    #
    #                                               M A P P I N G  M E T H O D S
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                   __I N P U T C O M M A N D T O T H R U S T
    #
    # For a given thrust command, it converts it into a force [N] (lift force) to be sent in Gazebo simulation.
    # ------------------------------------------------------------------------------------------------------------------

    def __inputCommandToThrust(self, input):
        totalLift = A2_THRUST_LIFT * (input ** 2) + A1_THRUST_LIFT * input + A0_THRUST_LIFT   # [N]
        return totalLift

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                               __I N P U T C O M M A N D T O V E L O C I T Y
    #
    # For a given thrust command, it converts it into the rotating speed [rad/s] to be sent in Gazebo simulation.
    # ------------------------------------------------------------------------------------------------------------------

    def __inputCommandToVelocity(self, input):
        rotatingSpeed = A1_THRUST_ROTATING_SPEED * input + A0_THRUST_ROTATING_SPEED # [rad/s]
        return rotatingSpeed

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                       __I N P U T C O M M A N D T O T O R Q U E
    #
    # For a given input command, it converts it into the torque [Nm] to be sent in Gazebo simulation (due to drag
    # force acting on the rotating propeller); this torque is considered in static condition.
    # ------------------------------------------------------------------------------------------------------------------
    def __inputCommandToTorque(self, input):
        # Calculating the lift value [N]:
        lift = self.__inputCommandToThrust(input)

        # Calcualting the torque value [Nm]:
        torque = A1_LIFT_TORQUE * lift + A0_LIFT_TORQUE

        return torque




class MotorControllerSim:
    # ==================================================================================================================
    #
    #                                               C O N S T R U C T O R
    #
    # This class completely handles all for virtual Crazyflie propellers/motors.
    # INPUTS:
    #   1) cfName -> name of the crazyflie in the simulation;
    # ==================================================================================================================
    def __init__(self, cfName):
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                           P R O P E R T I E S  I N I T I A L I Z A T I O N
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Name of the virtual Crazyflie:
        self.cfName = cfName

        # Variable to decide wether to send or not commands to the motors:
        self.canSend = False

        # Motors instantiation:
        self.M1 = MotorSim(cfName, 1, rotatingDirection.CCW)
        self.M2 = MotorSim(cfName, 2, rotatingDirection.CW)
        self.M3 = MotorSim(cfName, 3, rotatingDirection.CCW)
        self.M4 = MotorSim(cfName, 4, rotatingDirection.CW)

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                   S U B S C R I B E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Subscriber that receives motor commands coming from MotionCommanderSim:
        self.motor_command_sub = rospy.Subscriber('/' + cfName + '/motor_command', Attitude,
                                                  self.__motor_command_sub_callback)

    # ==================================================================================================================
    #
    #                                           C A L L B A C K  M E T H O D S
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                  __M O T O R C O M M A N D S U B C A L L B A C K
    #
    # This callback is called everytime a motor command is sent from the MotionCommanderSim; it converts the commands
    # coming from the flight controller to input commands for the motors.
    # ------------------------------------------------------------------------------------------------------------------
    def __motor_command_sub_callback(self, msg):
        if self.canSend:
            # Extracting request info:
            roll = msg.desired_attitude.roll / 2.0
            pitch = msg.desired_attitude.pitch / 2.0
            yaw = msg.desired_attitude.yaw
            thrust = msg.desired_thrust

            # Calculating the thrust for each motor:
            thrust_M1 = constrain(thrust - roll - pitch - yaw, 0, MAX_THRUST)
            thrust_M2 = constrain(thrust - roll + pitch + yaw, 0, MAX_THRUST)
            thrust_M3 = constrain(thrust + roll + pitch - yaw, 0, MAX_THRUST)
            thrust_M4 = constrain(thrust + roll - pitch + yaw, 0, MAX_THRUST)

            # Sending the thrust command:
            self.M1.sendInputCommand(thrust_M1)
            self.M2.sendInputCommand(thrust_M2)
            self.M3.sendInputCommand(thrust_M3)
            self.M4.sendInputCommand(thrust_M4)
        else:
            self.stopMotors()

    # ==================================================================================================================
    #
    #                                            M E T H O D S
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                           S T O P M O T O R S
    #
    # This method is used to stop all the motors.
    # ------------------------------------------------------------------------------------------------------------------
    def stopMotors(self):
        self.M1.stopMotor()
        self.M2.stopMotor()
        self.M3.stopMotor()
        self.M4.stopMotor()

    # ==================================================================================================================
    #
    #                                               D E B U G  M E T H O D S
    #
    # ==================================================================================================================
    def sendManualCommand(self, thrustCmd, rollCmd, pitchCmd, yawCmd):
        # Extracting request info:
        roll = rollCmd / 2.0
        pitch = pitchCmd / 2.0
        yaw = yawCmd
        thrust = thrustCmd

        # Calculating the thrust for each motor:
        thrust_M1 = constrain(thrust - roll + pitch + yaw, 0, MAX_THRUST)
        thrust_M2 = constrain(thrust - roll - pitch - yaw, 0, MAX_THRUST)
        thrust_M3 = constrain(thrust + roll - pitch + yaw, 0, MAX_THRUST)
        thrust_M4 = constrain(thrust + roll + pitch - yaw, 0, MAX_THRUST)

        print('INPUT COMMANDS: ', thrust_M1, '; ', thrust_M2, '; ', thrust_M3, '; ', thrust_M4)

        # Sending the thrust command:
        self.M1.sendInputCommand(thrust_M1)
        self.M2.sendInputCommand(thrust_M2)
        self.M3.sendInputCommand(thrust_M3)
        self.M4.sendInputCommand(thrust_M4)