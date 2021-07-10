# ROS MODULES
import rospy
import actionlib
# CUSTOM MODULES

from crazy_common_py.dataTypes import Vector3
from crazyflie_simulator.MotorControllerSim import MotorControllerSim
from crazy_common_py.common_functions import rad2deg

# OTHER MODULES
import time
import math

# MESSAGES
# Topic
from crazyflie_messages.msg import Position, CrazyflieState, Attitude
from std_msgs.msg import Empty

# Service
from crazyflie_messages.srv import Takeoff_srv, Takeoff_srvResponse, Takeoff_srvRequest

# Action
from crazyflie_messages.msg import TakeoffAction, TakeoffGoal, TakeoffResult, TakeoffFeedback

# CONSTANTS
# Takeoff:
DEFAULT_TAKEOFF_HEIGHT = 0.2    # [m]
DEFAULT_TAKEOFF_SPEED = 0.5     # [m/s]
DEFAULT_LAND_HEIGHT = 0.15

class MotionCommanderSim:
    # ==================================================================================================================
    #
    #                                               C O N S T R U C T O R
    #
    # This class is similar to the MotionCommander of official Bitcraze Python library: it collects high level commands.
    # INPUTS:
    #   1) cfName -> name of the crazyflie in the simulation;
    # ==================================================================================================================
    def __init__(self, cfName):
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                           P R O P E R T I E S  I N I T I A L I Z A T I O N
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Name of the virtual Crazyflie:
        self.name = cfName

        # Property to understand if the Crazyflie is flying:
        self.isFlying = False

        # Instance of motor controller:
        self.motors_controller = MotorControllerSim(cfName)

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                    S U B S C R I B E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Subscribers to perform operations at a certain frequency:
        self.pace_100Hz_sub = rospy.Subscriber('/pace_100Hz', Empty, self.__pace_100Hz_callback)
        self.pace_500Hz_sub = rospy.Subscriber('/pace_500Hz', Empty, self.__pace_500Hz_callback)

        # Subscriber to obtain the actual state of the virtual Crazyflie:
        self.actual_state_sub = rospy.Subscriber('/' + cfName + '/state', CrazyflieState,
                                                 self.__actual_state_sub_callback)
        self.actual_state = CrazyflieState()

        # Subscriber to obtain the desired motor command (final output of all the pids within FlightControllerSim):
        self.desired_motor_command_sub = rospy.Subscriber('/' + cfName + '/set_desired_motor_command', Attitude,
                                                          self.__desired_motor_command_callback)
        self.desired_motor_command = Attitude()

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       P U B L I S H E R S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Publisher to publish the desired destination (to be sent to FlightControllerSim):
        self.trajectory_pub = rospy.Publisher('/' + cfName + '/set_destination_position', Position, queue_size=1)
        self.position_target = Position()

        # Publisher to publish motor commands directly to the motors:
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # NOTE: "/set_destination_position" keeps publishing the desired position, and FlightControllerSim keeps
        # calculating the motor commands publishing them on "/set_desired_motion_command"; anyway it's the
        # MotionCommander that decides when to actually send commands to the motors.
        # TODO: FORSE INUTILE, NON PIU USATO
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        self.motor_command_pub = rospy.Publisher('/' + cfName + '/motor_command', Attitude, queue_size=1)
        self.motor_command = Attitude()

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                           S E R V I C E S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Takeoff service (server + client):
        self.takeoff_srv = rospy.Service('/' + cfName + '/takeoff', Takeoff_srv, self.__takeoff_srv_callback)
        self.takeoff_srv_client = rospy.ServiceProxy('/' + cfName + '/takeoff', Takeoff_srv)

        # Land service:
        self.land_srv = rospy.Service('/' + cfName + '/land', Takeoff_srv, self.__takeoff_srv_callback)
        self.land_srv_client = rospy.ServiceProxy('/' + cfName + '/land', Takeoff_srv)

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                           A C T I O N S  S E T U P
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Takeoff action (server + client):
        self.takeoff_act = actionlib.SimpleActionServer('/' + cfName + '/' + 'takeoff_actn', TakeoffAction,
                                                        self.__takeoff_act_callback, False)
        self.takeoff_act.start()
        self.takeoff_act_client = actionlib.SimpleActionClient('/' + cfName + '/' + 'takeoff_actn', TakeoffAction)

        

    # ==================================================================================================================
    #
    #                                     C A L L B A C K  M E T H O D S  (T O P I C S)
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                  __P A C E _ 1 0 0 H Z _ C A L L B A C K
    #
    # This callback is called every 1 / 100 seconds, is used to publish the desired position to the FlightController at
    # 100 Hz.
    # ------------------------------------------------------------------------------------------------------------------
    def __pace_100Hz_callback(self, msg):
        if self.isFlying:
            self.trajectory_pub.publish(self.position_target)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                  __P A C E _ 5 0 0 H Z _ C A L L B A C K
    #
    # This callback is called every 1 / 500 seconds, is used to publish the desired motor command input to the motors
    # at 500 Hz.
    # ------------------------------------------------------------------------------------------------------------------
    def __pace_500Hz_callback(self, msg):
        if self.isFlying:
            self.motor_command_pub.publish(self.desired_motor_command)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                       __D E S I R E D _ M O T O R _ C O M M A N D _ C A L L B A C K
    #
    # This callback saves the motor commands coming from the FLightControllerSim in a class' variable.
    # ------------------------------------------------------------------------------------------------------------------
    def __desired_motor_command_callback(self, msg):
        self.desired_motor_command.desired_attitude.roll = msg.desired_attitude.roll
        self.desired_motor_command.desired_attitude.pitch = msg.desired_attitude.pitch
        self.desired_motor_command.desired_attitude.yaw = msg.desired_attitude.yaw
        self.desired_motor_command.desired_thrust = msg.desired_thrust

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                               __M O T O R _ C O M M A N D _ C A L L B A C K
    #
    # ------------------------------------------------------------------------------------------------------------------
    def __motor_command_sub_callback(self, msg):
        pass

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                               __A C T U A L _ S T A T E _ S U B _ C A L L B A C K
    #
    # This callback updates the internal variable reffering to actual state of the Crazyflie.
    # ------------------------------------------------------------------------------------------------------------------
    def __actual_state_sub_callback(self, msg):
        self.actual_state.position.x = msg.position.x
        self.actual_state.position.y = msg.position.y
        self.actual_state.position.z = msg.position.z

        self.actual_state.orientation.roll = msg.orientation.roll
        self.actual_state.orientation.pitch = msg.orientation.pitch
        self.actual_state.orientation.yaw = msg.orientation.yaw

        self.actual_state.velocity.x = msg.velocity.x
        self.actual_state.velocity.y = msg.velocity.y
        self.actual_state.velocity.z = msg.velocity.z

        self.actual_state.rotating_speed.x = msg.rotating_speed.x
        self.actual_state.rotating_speed.y = msg.rotating_speed.y
        self.actual_state.rotating_speed.z = msg.rotating_speed.z

    # ==================================================================================================================
    #
    #                                 C A L L B A C K  M E T H O D S  (S E R V I C E S)
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                    __T A K E O F F _ S R V _ C A L L B A C K
    #
    # ------------------------------------------------------------------------------------------------------------------
    def __takeoff_srv_callback(self, request):
        # Getting takeoff height:
        takeoff_height = request.takeoff_height

        # Getting initial position:
        initial_position = Vector3(self.actual_state.position.x, self.actual_state.position.y, self.actual_state.position.z)
        initial_attitude = rad2deg(self.actual_state.orientation.yaw)

        # Setting up output:
        response = Takeoff_srvResponse()
        response.result = False

        # Returning true when the Crazyflie has reached the desired height:
        rate = rospy.Rate(100)
        while True:
            # Getting current state:
            actual_state = self.actual_state

            # Verify if the Crazyflie has reached the takeoff height:
            if math.fabs(takeoff_height - actual_state.position.z) <= 0.005:
                response.result = True
                self.isFlying = True
                break
            # Sending commands to reach the takeoff height:
            self.position_target.desired_position.x = initial_position.x
            self.position_target.desired_position.y = initial_position.y
            self.position_target.desired_position.z = takeoff_height
            self.position_target.desired_yaw = initial_attitude
            self.trajectory_pub.publish(self.position_target)

            self.motor_command_pub.publish(self.desired_motor_command)

            rate.sleep()

        return response

    # ==================================================================================================================
    #
    #                                 C A L L B A C K  M E T H O D S  (A C T I O N S)
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                    __T A K E O F F _ A C T _ C A L L B A C K
    #
    # ------------------------------------------------------------------------------------------------------------------
    def __takeoff_act_callback(self, goal):
        # Rate definition:
        rate = rospy.Rate(100.0)
        success = True

        # Output:
        feedback = TakeoffFeedback()
        result = TakeoffResult()

        # Getting initial position:
        initial_position = Vector3(self.actual_state.position.x, self.actual_state.position.y,
                                   self.actual_state.position.z)
        initial_attitude = rad2deg(self.actual_state.orientation.yaw)

        # Getting target takeoff height:
        takeoff_height = goal.takeoff_height

        while True:
            # Getting current state:
            actual_state = self.actual_state

            # Aboslute distance:
            absolute_distance = math.fabs(takeoff_height - actual_state.position.z)

            # Publishing absolute distance as feedback:
            feedback.absolute_distance = absolute_distance
            self.takeoff_act.publish_feedback(feedback)

            # Verify if the Crazyflie has reached the takeoff height:
            if absolute_distance <= 0.005:
                self.isFlying = True
                success = True
                break

            # Check preemption:
            if self.takeoff_act.is_preempt_requested():
                success = False
                info_msg = 'Takeoff action canceled for ' + self.name
                rospy.loginfo(info_msg)
                result.result = False
                self.takeoff_act.set_preempted()
                break

            # Sending commands to reach the takeoff height:
            self.position_target.desired_position.x = initial_position.x
            self.position_target.desired_position.y = initial_position.y
            self.position_target.desired_position.z = takeoff_height
            self.position_target.desired_yaw = initial_attitude
            self.trajectory_pub.publish(self.position_target)

            self.motor_command_pub.publish(self.desired_motor_command)

            rate.sleep()

        if success:
            result.result = True
            self.takeoff_act.set_succeeded(result)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                           __T A K E O F F _ A C T _ C L I E N T _ F E E D B A C K _ C B
    #
    # Callback function for the feedback of takeoff action client.
    # ------------------------------------------------------------------------------------------------------------------
    def __takeoff_act_client_feedback_cb(self, feedback):
        message = self.name + ' is taking off; actual absolute distance from target height: ' + str(feedback.absolute_distance)
        rospy.logdebug(message)

    # ==================================================================================================================
    #
    #                                           B A S I C  M E T H O D S
    #
    # ==================================================================================================================
    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                             T A K E O F F _ S R V
    #
    # This methods is used to perform a takeoff with service: if there are multiple crazyflies in the scene, they will
    # takeoff once at time.
    # INPUTS:
    #   - height -> z coordinate [m] where the Crazyflie will move during the takeoff;
    # ------------------------------------------------------------------------------------------------------------------
    def takeoff_srv(self, height=DEFAULT_TAKEOFF_HEIGHT):
        # Setting up the takeoff request:
        request = Takeoff_srvRequest()
        request.takeoff_height = height

        # Requesting takeoff:
        self.takeoff_srv_client(request)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                             T A K E O F F _ A C T N
    #
    # This methods is used to perform a takeoff action: if there are multuple crazyflies in the scene, they will
    # takeoff all (almost) at the same time.
    # INPUTS:
    #   - height -> z coordinate [m] where the Crazyflie will move during the takeoff;
    # ------------------------------------------------------------------------------------------------------------------
    def takeoff_actn(self, height=DEFAULT_TAKEOFF_HEIGHT):
        # Setting up takeoff request:
        goal = TakeoffGoal()
        goal.takeoff_height = height

        # Action requesting;
        self.takeoff_act_client.send_goal(goal, feedback_cb=self.__takeoff_act_client_feedback_cb)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                                L A N D
    #
    # This methods is used to perform a landing.
    # INPUTS:
    #   - height -> z coordinate [m] where the Crazyflie will move and stops its motors;
    # ------------------------------------------------------------------------------------------------------------------
    def land(self, height=DEFAULT_LAND_HEIGHT):
        actual_state = self.actual_state
        self.go_to(Vector3(actual_state.position.x, actual_state.position.y, height))

    # ------------------------------------------------------------------------------------------------------------------
    #
    #                                               G O _ T O
    #
    # This methods is used to set a destination.
    # INPUTS:
    #   - destination -> Vector3 representing coordinates of destination point [m];
    #   - yaw -> yaw value at destination [deg]
    # ------------------------------------------------------------------------------------------------------------------
    def go_to(self, destination=Vector3(), yaw=0.0):
        if self.isFlying == True:
            self.position_target.desired_position.x = destination.x
            self.position_target.desired_position.y = destination.y
            self.position_target.desired_position.z = destination.z
            self.position_target.desired_yaw = yaw
            self.trajectory_pub.publish(self.position_target)

            self.motor_command_pub.publish(self.desired_motor_command)
        else:
            error_message = "Crazyflie " + self.name + " has received a destination, but it's not flying! Please takeoff first!"
            rospy.logerr(error_message)



