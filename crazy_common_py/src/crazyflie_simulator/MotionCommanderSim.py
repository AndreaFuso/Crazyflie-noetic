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

        self.OK = False

        #TODO: in FlightControllerSim bisogna mettere anche il yaw desired come input!!


        self.position_target.desired_position.x = self.actual_state.position.x
        self.position_target.desired_position.y = self.actual_state.position.y
        self.position_target.desired_position.z = self.actual_state.position.z

    def __pace_100Hz_callback(self, msg):
        self.trajectory_pub.publish(self.position_target)

    def __pace_500Hz_callback(self, msg):
        if self.OK:
            self.motor_command_pub.publish(self.desired_motor_command)

    def __desired_motor_command_callback(self, msg):
        self.desired_motor_command.desired_attitude.roll = msg.desired_attitude.roll
        self.desired_motor_command.desired_attitude.pitch = msg.desired_attitude.pitch
        self.desired_motor_command.desired_attitude.yaw = msg.desired_attitude.yaw
        self.desired_motor_command.desired_thrust = msg.desired_thrust

    def __motor_command_sub_callback(self, msg):
        pass

    def __actual_state_sub_callback(self, msg):
        self.actual_state = msg



    def takeoff(self, height=DEFAULT_TAKEOFF_HEIGHT, speed=DEFAULT_TAKEOFF_SPEED):
        pass

    def go_to(self, destination=Vector3()):
        time.sleep(5)
        self.position_target.desired_position.x = destination.x
        self.position_target.desired_position.y = destination.y
        self.position_target.desired_position.z = destination.z
        self.trajectory_pub.publish(self.position_target)

        self.motor_command_pub.publish(self.desired_motor_command)
        self.OK = True



