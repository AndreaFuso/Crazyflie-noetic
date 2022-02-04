#! /usr/bin/env python3
import rospy
from casadi import *
import numpy as np
import cvxpy as cp      # cvxpy is a toolbox for convex optimization in python, 
                        # we need to solve a convex QP
from crazyflie_simulator.CrazySim import CrazySim
from crazyflie_manager.CrazyManager import *
from crazy_common_py.dataTypes import Vector3
from crazy_common_py.common_functions import rad2deg, deg2rad
from std_msgs.msg import Empty
from crazyflie_messages.msg import Position, CrazyflieState, Attitude


class CBF_controller():

    def __init__(self, v_lim, alpha, x_goal, x, d_lim):
        
        self.x = x
        self.x_goal = x_goal
        self.distance = np.linalg.norm(self.x_goal - self.x)
        self.K      = v_lim/self.distance
        if self.distance < d_lim:
            self.K = v_lim/d_lim
        self.alpha  = alpha


    def set_obstacle(self, x_obs, r_obs):

        self.h = lambda x1,x2 : ((x1-x_obs[0])**2 + (x2-x_obs[1])**2 
                                - r_obs**2)*0.5
        self.grad_h = lambda x1,x2 : np.array([x1-x_obs[0] , x2-x_obs[1]])

    def get_v_des(self, x):
        # Given current state return proportional nominal controller
        v_des = - self.K * (x - self.x_goal)

        return v_des

    def get_cbf_v(self, x):

        # Given current state solve pointwise Quadratic Program and get safe 
        # control action (velocity)

        v_opt      = cp.Variable(2)
        h_num      = self.h(x[0],x[1])
        grad_h_num = self.grad_h(x[0],x[1])
        v_des  = self.get_v_des(x)

        # Solve QP for u_opt
        obj         = cp.Minimize((1/2)*cp.quad_form(v_opt-v_des,np.eye(2)))
        constraints = [grad_h_num.T @ v_opt >= -self.alpha*h_num ] 
        prob = cp.Problem(obj,constraints)
        prob.solve()

        return v_opt.value



def cbf_target_sub_callback(msg):
    cbf_target.desired_position.x = msg.desired_position.x
    cbf_target.desired_position.y = msg.desired_position.y
    cbf_target.desired_position.z = msg.desired_position.z

    return cbf_target

def state_sub_callback(msg):
    actual_state.position.x = msg.position.x
    actual_state.position.y = msg.position.y
    actual_state.position.z = msg.position.z

    actual_state.velocity.x = msg.velocity.x
    actual_state.velocity.y = msg.velocity.y
    actual_state.velocity.z = msg.velocity.z

    actual_state.orientation.roll = rad2deg(msg.orientation.roll)
    actual_state.orientation.pitch = rad2deg(msg.orientation.pitch)
    actual_state.orientation.yaw = rad2deg(msg.orientation.yaw)

    actual_state.rotating_speed.x = rad2deg(msg.rotating_speed.x)
    actual_state.rotating_speed.y = rad2deg(msg.rotating_speed.y)
    actual_state.rotating_speed.z = rad2deg(msg.rotating_speed.z)

    return actual_state



if __name__ == '__main__':
    
    # Setting the parameters for the cbf controller and the position goal
    v_lim = 0.5
    alpha = 1
    d_lim = 0.4

    # Defining obstacle geometry
    x_obs = np.array([1.0, 0.1])
    r_obs = 0.30
    r_drone = 0.05
    r_safety = 0.05
    r_tot = r_obs + r_drone + r_safety

    # Publisher setup:
    # Publisher to publish the target velocity (output of nlp)
    cbf_velocity = Position()
    cbf_velocity_pub = rospy.Publisher('/cf1/mpc_velocity', Position, queue_size=1)


    # Publisher to publish the target position (when the drone is close to target)
    mpc_switch_pub = rospy.Publisher('/cf1/mpc_switch', Position, queue_size=1)

    # Subscribers setup
    # Subscriber to get the mpc target position
    cbf_target_sub = rospy.Subscriber('/cf1/cbf_target', Position, cbf_target_sub_callback)

    # Subscriber to get the actual state of the drone in the simulation:
    state_sub = rospy.Subscriber('/cf1/state', CrazyflieState, state_sub_callback)
    actual_state = CrazyflieState()

    # Initializing the mpc_target_init to start the mpc controller only 
    # when someone publishes on /cf1/mpc_target
    cbf_target = Position()
    cbf_target_init = Position()
    cbf_target.desired_position.x = actual_state.position.x
    cbf_target.desired_position.y = actual_state.position.y
    cbf_target.desired_position.z = actual_state.position.z
    cbf_target_init.desired_position.x = cbf_target.desired_position.x
    cbf_target_init.desired_position.y = cbf_target.desired_position.y
    cbf_target_init.desired_position.z = cbf_target.desired_position.z


    
    # Node initialization:
    rospy.init_node('node_cbf_2d', log_level=rospy.DEBUG)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if (cbf_target.desired_position.x == cbf_target_init.desired_position.x 
            and cbf_target.desired_position.y == cbf_target_init.desired_position.y 
            and cbf_target.desired_position.z == cbf_target_init.desired_position.z):
            # This is needed to wait for the cbf_target to be published
            pass

        else:
            # Getting the new goal
            x_goal = np.array([cbf_target.desired_position.x, cbf_target.desired_position.y])
            # Setting initial position at the current time step
            x0 = np.array([actual_state.position.x, actual_state.position.y])
            # Creating the instance of the CBF controller
            cbf_controller = CBF_controller(v_lim, alpha, x_goal, x0, d_lim)
            # Setting obstacles
            cbf_controller.set_obstacle(x_obs, r_tot)
            # Getting cbf velocity
            v = cbf_controller.get_cbf_v(x0)
            # Setting cbf_velocity msg to be published on /cf1/mpc_velocity
            cbf_velocity.desired_velocity.x = v[0]
            cbf_velocity.desired_velocity.y = v[1]
            cbf_velocity_pub.publish(cbf_velocity)

            # We update this so that we can publish any target we want
            cbf_target_init.desired_position.x = actual_state.position.x
            cbf_target_init.desired_position.y = actual_state.position.y
            cbf_target_init.desired_position.z = actual_state.position.z

            # while (abs(cbf_target.desired_position.x - actual_state.position.x) < 0.3 and 
            #        abs(cbf_target.desired_position.y - actual_state.position.y) < 0.3):
            #     mpc_switch_pub.publish(cbf_target)

        rate.sleep()

    rospy.spin()