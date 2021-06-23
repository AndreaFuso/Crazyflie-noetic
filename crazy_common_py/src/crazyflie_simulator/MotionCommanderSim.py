# ROS MODULES
import rospy

# CUSTOM MODULES
from crazy_common_py.dataTypes import Vector3

# OTHER MODULES
import time

# MESSAGES
from crazyflie_messages.msg import Position, CrazyflieState, Attitude

# CONSTANTS
# Takeoff:
DEFAULT_TAKEOFF_HEIGHT = 0.2    # [m]
DEFAULT_TAKEOFF_SPEED = 0.5     # [m/s]

class MotionCommanderSim:
    def __init__(self, name):
        self.name = name



        self.trajectory_pub = rospy.Publisher('/' + name + '/set_destination_position', Position, queue_size=1)
        self.position_target = Position()

        self.actual_state_sub = rospy.Subscriber('/' + name + '/state', CrazyflieState, self.__actual_state_sub_callback)
        self.actual_state = CrazyflieState()

        #TODO: fare in modo che quando parta i motori siano fermi, si puo mettere un servizio



        time.sleep(3)

        self.position_target.desired_position.x = self.actual_state.position.x
        self.position_target.desired_position.y = self.actual_state.position.y
        self.position_target.desired_position.z = self.actual_state.position.z


    def __motor_command_sub_callback(self, msg):
        pass

    def __actual_state_sub_callback(self, msg):
        self.actual_state = msg
        self.trajectory_pub.publish(self.position_target)

    def takeoff(self, height=DEFAULT_TAKEOFF_HEIGHT, speed=DEFAULT_TAKEOFF_SPEED):
        pass

    def go_to(self, destination=Vector3()):
        self.position_target.desired_position.x = destination.x
        self.position_target.desired_position.y = destination.y
        self.position_target.desired_position.z = destination.z
        self.trajectory_pub.publish(self.position_target)
