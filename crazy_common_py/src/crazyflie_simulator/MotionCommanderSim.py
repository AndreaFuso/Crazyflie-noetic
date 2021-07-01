# ROS MODULES
import rospy

# CUSTOM MODULES
from crazy_common_py.dataTypes import Vector3

# OTHER MODULES
import time

# MESSAGES
from crazyflie_messages.msg import Position, CrazyflieState, Attitude
from std_msgs.msg import Empty


# CONSTANTS
# Takeoff:
DEFAULT_TAKEOFF_HEIGHT = 0.2    # [m]
DEFAULT_TAKEOFF_SPEED = 0.5     # [m/s]
DEFAULT_LAND_HEIGHT = 0.15

class MotionCommanderSim:
    def __init__(self, name):
        self.name = name

        self.pace_100Hz_sub = rospy.Subscriber('/pace_100Hz', Empty, self.__pace_100Hz_callback)
        self.pace_500Hz_sub = rospy.Subscriber('/pace_500Hz', Empty, self.__pace_500Hz_callback)

        self.trajectory_pub = rospy.Publisher('/' + name + '/set_destination_position', Position, queue_size=1)
        self.position_target = Position()

        self.motor_command_pub = rospy.Publisher('/' + name + '/motor_command', Attitude, queue_size=1)
        self.motor_command = Attitude()

        self.actual_state_sub = rospy.Subscriber('/' + name + '/state', CrazyflieState, self.__actual_state_sub_callback)
        self.actual_state = CrazyflieState()

        self.desired_motor_command_sub = rospy.Subscriber('/' + name + '/set_desired_motor_command', Attitude, self.__desired_motor_command_callback)
        self.desired_motor_command = Attitude()

        self.destination_DEBUG_sub = rospy.Subscriber('/' + name + '/DEBUG_destination_publisher', Position,
                                                      self.__DEBUG_destination_callback)

        self.OK = False

        #TODO: in FlightControllerSim bisogna mettere anche il yaw desired come input!!


        self.position_target.desired_position.x = self.actual_state.position.x
        self.position_target.desired_position.y = self.actual_state.position.y
        self.position_target.desired_position.z = self.actual_state.position.z

    def __pace_100Hz_callback(self, msg):
        if self.OK:
            self.trajectory_pub.publish(self.position_target)

    def __pace_500Hz_callback(self, msg):
        if self.OK:
            self.motor_command_pub.publish(self.desired_motor_command)

    def __DEBUG_destination_callback(self, msg):
        self.OK = False
        self.position_target.desired_position.x = msg.desired_position.x
        self.position_target.desired_position.y = msg.desired_position.y
        self.position_target.desired_position.z = msg.desired_position.z
        self.trajectory_pub.publish(self.position_target)
        self.OK = True

    def __desired_motor_command_callback(self, msg):
        self.desired_motor_command.desired_attitude.roll = msg.desired_attitude.roll
        self.desired_motor_command.desired_attitude.pitch = msg.desired_attitude.pitch
        self.desired_motor_command.desired_attitude.yaw = msg.desired_attitude.yaw
        self.desired_motor_command.desired_thrust = msg.desired_thrust

    def __motor_command_sub_callback(self, msg):
        pass

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



    def takeoff(self, height=DEFAULT_TAKEOFF_HEIGHT, speed=DEFAULT_TAKEOFF_SPEED):
        # Getting actual position:
        actual_state = self.actual_state
        time.sleep(5)
        self.OK = True
        self.go_to(Vector3(actual_state.position.x, actual_state.position.y, height))

    def prepareLanding(self, height=DEFAULT_LAND_HEIGHT):
        actual_state = self.actual_state
        self.go_to(Vector3(actual_state.position.x, actual_state.position.y, height))

    def go_to(self, destination=Vector3()):
        if self.OK == True:
            self.position_target.desired_position.x = destination.x
            self.position_target.desired_position.y = destination.y
            self.position_target.desired_position.z = destination.z
            self.trajectory_pub.publish(self.position_target)

            self.motor_command_pub.publish(self.desired_motor_command)



