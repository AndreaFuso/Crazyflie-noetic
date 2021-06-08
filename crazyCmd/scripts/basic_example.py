#! /usr/bin/env python3
# Importing ROS modules
import rospy

# other modules:
import time
# Importing messages:
from std_srvs.srv import Empty, EmptyRequest
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, Vector3, Quaternion
from crazyflie_messages.msg import CustomTrajectoryPointYaw_msg
from crazyflie_messages.srv import CustomTrajectoryPointYaw_srv, CustomTrajectoryPointYaw_srvRequest, Takeoff_srv, Takeoff_srvRequest

if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('basic_example')

    # ===========================================================================================
    #
    #                      U N P A U S E  G A Z E B O  S I M U L A T I O N
    #
    # ===========================================================================================
    # Waiting for service to unpause physics to be running:
    rospy.wait_for_service('/gazebo/unpause_physics')

    # Set up service to unpause Gazebo:
    unpause_gazebo_service = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

    # Creating the message to be sent to trigger the service:
    unpause_gazebo_service_msg = EmptyRequest()

    # Calling the service:
    result = unpause_gazebo_service(unpause_gazebo_service_msg)

    # ===========================================================================================
    #
    #                      T R A J E C T O R Y  D E F I N I T I O N
    #
    # ===========================================================================================
    # Creating a MultiDOFJoint Trajectory message
    # -------------------------------------------
    #           MultiDOFJointTrajectory
    # -------------------------------------------
    # # The header is used to specify the coordinate frame and the reference time for the trajectory durations
    # Header header
    #
    # # A representation of a multi-dof joint trajectory (each point is a transformation)
    # # Each point along the trajectory will include an array of positions/velocities/accelerations
    # # that has the same length as the array of joint names, and has the same order of joints as
    # # the joint names array.
    #
    # string[] joint_names
    # MultiDOFJointTrajectoryPoint[] points
    # -------------------------------------------
    #        MultiDOFJointTrajectoryPoint
    # -------------------------------------------
    # # Each multi-dof joint can specify a transform (up to 6 DOF)
    # geometry_msgs/Transform[] transforms
    #
    # # There can be a velocity specified for the origin of the joint
    # geometry_msgs/Twist[] velocities
    #
    # # There can be an acceleration specified for the origin of the joint
    # geometry_msgs/Twist[] accelerations
    # -------------------------------------------
    #                Transform
    # -------------------------------------------
    # # This represents the transform between two coordinate frames in free space.
    #
    # Vector3 translation
    # Quaternion rotation

    '''# Waiting for service to be on:
    rospy.wait_for_service('/go_to')

    # Service instantiation:
    trajectory_service = rospy.ServiceProxy('/go_to', CustomTrajectoryPointYaw_srv)

    # Message 1:
    my_trajectory = CustomTrajectoryPointYaw_srvRequest()
    my_trajectory.trajectory_point.point.x = 0.0
    my_trajectory.trajectory_point.point.y = 0.0
    my_trajectory.trajectory_point.point.z = 1.0
    my_trajectory.trajectory_point.yaw = 0.0

    response = trajectory_service(my_trajectory)
    rospy.loginfo("CHIAMATO SERVIZIO!! RISULTATO=%s", str(response.result))

    # Message 2:
    my_trajectory.trajectory_point.point.x = 1.0
    my_trajectory.trajectory_point.point.y = 1.0
    my_trajectory.trajectory_point.point.z = 1.0
    my_trajectory.trajectory_point.yaw = 0.0

    response = trajectory_service(my_trajectory)
    rospy.loginfo("CHIAMATO SERVIZIO!! RISULTATO=%s", str(response.result))'''

    # Wait for takeoff service to be online:
    rospy.wait_for_service('/takeoff')

    # Takeoff service instance:
    takeoff_service = rospy.ServiceProxy('/takeoff', Takeoff_srv)

    # Message:
    takeoff_request = Takeoff_srvRequest()

    max_PWM = 8163.0
    min_PWM = 5156.0

    delta_PWM = max_PWM - min_PWM

    thrust_perc = 0.57

    pwm = min_PWM + thrust_perc * delta_PWM

    takeoff_request.takeoff_settings.takeoff_height = 0.2
    takeoff_request.takeoff_settings.vertical_speed = pwm

    # Calling the service:
    takeoff_response = takeoff_service(takeoff_request)
    rospy.loginfo("TAKEOFF SERVICE responded: RESULT = %s", str(takeoff_response.result))

    rospy.spin()




