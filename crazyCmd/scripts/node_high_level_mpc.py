#! /usr/bin/env python3
import rospy
import time
from casadi import *
import numpy as np
from crazyflie_simulator.CrazySim import CrazySim
from crazyflie_manager.CrazyManager import *
from crazy_common_py.dataTypes import Vector3
from crazy_common_py.common_functions import rad2deg, deg2rad
from std_msgs.msg import Empty
from crazyflie_messages.msg import Position, CrazyflieState, Attitude
from crazy_common_py.common_functions import standardNameList
from crazyflie_swarm.CrazySwarmSim import CrazySwarmSim


# def nlp_solver_2d(mpc_target, actual_state, x_obs, y_obs, r_obs, T_mpc, N_mpc, 
#                     r_drone, r_safety, x1_opt, x2_opt): #, v1_opt, v2_opt

#     # Getting the actual position to set the initial condition
#     x_pos = actual_state.position.x
#     y_pos = actual_state.position.y

#     # Setting the desired position
#     x_des = mpc_target.desired_position.x
#     y_des = mpc_target.desired_position.y

#     # Setting desired velocity
#     vx_des = (x_des-x_pos)/T_mpc
#     vy_des = (y_des-y_pos)/T_mpc
#     if vx_des > 0.5:
#         vx_des = 0.5
#     if vy_des > 0.5:
#         vy_des = 0.5

#     # Declare model variables
#     x1 = MX.sym('x1')
#     x2 = MX.sym('x2')
#     x = vertcat(x1, x2)

#     v1 = MX.sym('v1')
#     v2 = MX.sym('v2')
#     v = vertcat(v1, v2)

#     # Model equations
#     xdot = vertcat(v1, v2)

#     # Distance to be covered
#     vector = [x_des-x_pos, y_des-y_pos]
#     vector = np.array(vector)
#     distance = np.linalg.norm(vector)

#     # Defining weight for position cost
#     a_pos = 0.15
#     b_pos = -1.3
#     d_lb = 0.5
#     d_ub = 3
#     k_pos = a_pos*distance**(b_pos)
#     if distance < d_lb:
#         k_pos = a_pos*d_lb**(b_pos)
#     if distance > d_ub:
#         k_pos = a_pos*d_ub**(b_pos)

#     # Objective term (minimize control effort)
#     L = (v1-vx_des)**2 + (v2-vy_des)**2 + k_pos*(x1-x_des)**2 + k_pos*(x2-y_des)**2

#     # Formulate discrete time dynamics
#     if False:
#         # CVODES from the SUNDIALS suite
#         dae = {'x':x, 'p':u, 'ode':xdot, 'quad':L}
#         opts = {'tf':T/N}
#         F = integrator('F', 'cvodes', dae, opts)
#     else:
#         # Fixed step Runge-Kutta 4 integrator
#         M = 4 # RK4 steps per interval
#         DT = T_mpc/N_mpc/M
#         f = Function('f', [x, v], [xdot, L])
#         X0 = MX.sym('X0', 2)
#         U = MX.sym('U', 2)
#         X = X0
#         Q = 0
#         for j in range(M):
#             k1, k1_q = f(X, U)
#             k2, k2_q = f(X + DT/2 * k1, U)
#             k3, k3_q = f(X + DT/2 * k2, U)
#             k4, k4_q = f(X + DT * k3, U)
#             X = X + DT/6*(k1 + 2*k2 + 2*k3 + k4)
#             Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q)
#         F = Function('F', [X0, U], [X, Q],['x0','p'],['xf','qf'])
        
        
#     # MPC LOOP
#     P_0 = [x_pos, y_pos]
#     P_N = [x_des, y_des]

#     v_ig = [vx_des, vy_des]
    
#     # Initializing state estimate at time instant i
#     X_i = P_0
#     v_i = v_ig
#     # print('X_i: ', X_i)


#     # Start with an empty NLP at each time step
#     w = []
#     # print('w: ', w)
#     w0 = []
#     lbw = []
#     ubw = []
#     J = 0
#     g = []        # it includes the constraint for obstacle avoidance
#     lbg = []
#     ubg = []

#     # "Lift" initial conditions
#     Xk = MX.sym('X0', 2)
#     w += [Xk]

#     lbw += [X_i[0].__float__(), X_i[1].__float__()]
#     ubw += [X_i[0].__float__(), X_i[1].__float__()]
#     w0 += [X_i[0].__float__(), X_i[1].__float__()]

#     # Formulate the NLP
#     for k in range(N_mpc):
#         # New NLP variable for the control
#         Vk = MX.sym('V_' + str(k), 2)  # creating symbolic expression for the 
#                                     # new optimization variable
#         w   += [Vk]     
#         lbw += [-inf, -inf]
#         ubw += [ inf,  inf]
#         w0  += [v_i[0].__float__(), v_i[1].__float__()]


#         # Integrate till the end of the interval
#         Fk = F(x0=Xk, p=Vk)         # we call the integrator
#         Xk_end = Fk['xf']
#         J=J+Fk['qf']

#         # New NLP variable for state at end of interval
#         Xk = MX.sym('X_' + str(k+1), 2)
#         w   += [Xk]
#         lbw += [-inf, -inf]
#         ubw += [ inf,  inf]
#         w0  += [x1_opt[k+1], x2_opt[k+1]]

#         # Add equality constraint (continuity constraint for multiple shooting)
#         g   += [Xk_end-Xk]
#         lbg += [0, 0]
#         ubg += [0, 0]

#         # Add inequality constraint for obstacle avoidance
#         for ii in range(len(x_obs)): # start here
#             g   += [(Xk[0] - x_obs[ii])**2 + (Xk[1] - y_obs[ii])**2]
#             lbg += [(r_obs[ii] + r_drone + r_safety)**2]
#             ubg += [+inf]


#         if k == N_mpc - 1:
#             X_final = P_N
#             g += [Xk_end - X_final]
#             lbg += [-1, -1]
#             ubg += [ 1,  1]

#     prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}
#     solver = nlpsol('solver', 'ipopt', prob);

#     # Solve the NLP
#     sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
#     w_opt = sol['x'].full().flatten()

#     # Extracting the optimal control inputs
#     x1_opt = []
#     x2_opt = []
#     v1_opt = []
#     v2_opt = []
#     x1_opt = w_opt[0::4]
#     x2_opt = w_opt[1::4]
#     v1_opt = w_opt[2::4]
#     v2_opt = w_opt[3::4]

#     # Extracting the control inputs to be applied
#     v_opt_i = []
#     v_opt_i += [v1_opt[0]]
#     v_opt_i += [v2_opt[0]]
#     v_i = np.array(v_opt_i)
    
#     v_desired = v_i
    
#     mpc_velocity = Position()

#     mpc_velocity.desired_velocity.x = v_desired[0]
#     mpc_velocity.desired_velocity.y = v_desired[1]
#     mpc_velocity.desired_velocity.z = 0.0

#     return mpc_velocity, x1_opt, x2_opt

def mpc_target_sub_callback(msg):
    mpc_target.desired_position.x = msg.desired_position.x
    mpc_target.desired_position.y = msg.desired_position.y
    mpc_target.desired_position.z = msg.desired_position.z
    
    return mpc_target


def single_state_sub_callback(msg):
    actual_state_single.position.x = msg.position.x
    actual_state_single.position.y = msg.position.y
    actual_state_single.position.z = msg.position.z

    actual_state_single.velocity.x = msg.velocity.x
    actual_state_single.velocity.y = msg.velocity.y
    actual_state_single.velocity.z = msg.velocity.z

    actual_state_single.orientation.roll = rad2deg(msg.orientation.roll)
    actual_state_single.orientation.pitch = rad2deg(msg.orientation.pitch)
    actual_state_single.orientation.yaw = rad2deg(msg.orientation.yaw)

    actual_state_single.rotating_speed.x = rad2deg(msg.rotating_speed.x)
    actual_state_single.rotating_speed.y = rad2deg(msg.rotating_speed.y)
    actual_state_single.rotating_speed.z = rad2deg(msg.rotating_speed.z)

    return actual_state_single


# class mpcHighLevel:

#     def __init__(self, cf_names):
        
#         # Name of the virtual Crazyflie
#         self.cf_names = cf_names

#         pass

def make_mpc_velocity_publishers():
    for cf_name in cf_names:
        mpc_velocity_pub = rospy.Publisher('/' + cf_name + '/mpc_velocity', 
                                            Position, queue_size=1)
        mpc_velocity_publishers.append(mpc_velocity_pub)


def swarm_mpc_velocity_pub():
    index = 0
    for index, mpc_velocity_pub in enumerate(mpc_velocity_publishers):
        mpc_velocity_pub.publish(mpc_velocity[index])


def make_actual_state_subscribers():
    index = 0
    for index,cf_name in enumerate(cf_names):
        state_sub = rospy.Subscriber('/' + cf_name + '/state', 
                                     CrazyflieState, single_state_sub_callback)
        actual_state_subscribers.append(state_sub)



if __name__ == '__main__':
    
    # Node initialization:
    rospy.init_node('node_high_level_mpc', log_level=rospy.DEBUG)

    # Extracting rosparam informations (to understand the number of crazyflies):
    number_of_cfs = rospy.get_param('swarm_node/cfs_number')
    print(number_of_cfs)

    # Generate a standard list of names:
    cf_names = standardNameList(number_of_cfs)
    print(cf_names)

    # Instantiate a swarm:
    swarm = CrazySwarmSim(cf_names)


    # Time interval and number of control intervals for the MPC
    T_mpc = 5
    N_mpc = 5


    ###############################################################################

    #                     P U B L I S H E R S   S E T U P

    ###############################################################################

    # Publisher to publish the target velocity (output of nlp)

    mpc_velocity_single = Position()
    mpc_velocity = []

    for ii in range(number_of_cfs):
        mpc_velocity.append(mpc_velocity_single)

    # List of mpc_velocity Publishers
    mpc_velocity_publishers = []
    make_mpc_velocity_publishers()


    ###############################################################################

    #                     S U B S C R I B E R S   S E T U P

    ###############################################################################

    # Subscriber to get the mpc target position
    mpc_target_sub = rospy.Subscriber('/swarm/mpc_target', Position, mpc_target_sub_callback)
    mpc_target = Position()

    # Subscriber to get the actual state of the drones in the simulation:
    actual_state_single = CrazyflieState()
    actual_state = []

    for ii in range(number_of_cfs):
        actual_state.append(actual_state_single)
    
    # List of Subscribers
    actual_state_subscribers = []
    make_actual_state_subscribers()
    
    
    # state_sub = rospy.Subscriber('/cf1/state', CrazyflieState, state_sub_callback)
    # actual_state = CrazyflieState()


    # # Initializing the mpc_target_init to start the mpc controller only 
    # # when someone publishes on /cf1/mpc_target
    # mpc_target = Position()
    # mpc_target_init = Position()
    # mpc_target.desired_position.x = actual_state.position.x
    # mpc_target.desired_position.y = actual_state.position.y
    # mpc_target.desired_position.z = actual_state.position.z
    # mpc_target_old = mpc_target
    # mpc_target_init.desired_position.x = mpc_target.desired_position.x
    # mpc_target_init.desired_position.y = mpc_target.desired_position.y
    # mpc_target_init.desired_position.z = mpc_target.desired_position.z

    # # Initializing x1_opt, x2_opt to use them as initial guess for each step
    # x1_opt_old = np.linspace(actual_state.position.x, mpc_target.desired_position.x, N_mpc+1)
    # x2_opt_old = np.linspace(actual_state.position.y, mpc_target.desired_position.y, N_mpc+1)


    # # rate = rospy.Rate(10)

    rate = rospy.Rate(N_mpc/T_mpc)

    # while not rospy.is_shutdown():
        
    #     if (mpc_target_old != mpc_target):
    #         x1_opt_old = np.linspace(actual_state.position.x, mpc_target.desired_position.x, N_mpc+1)
    #         x2_opt_old = np.linspace(actual_state.position.y, mpc_target.desired_position.y, N_mpc+1)

    #     if (mpc_target.desired_position.x == mpc_target_init.desired_position.x 
    #         and mpc_target.desired_position.y == mpc_target_init.desired_position.y 
    #         and mpc_target.desired_position.z == mpc_target_init.desired_position.z):
    #         # This is needed to wait for the mpc_target to be published
    #         pass
    #     else:
    #         mpc_velocity, x1_opt, x2_opt = nlp_solver_2d(mpc_target, actual_state, 
    #             x_obs, y_obs, r_obs, T_mpc, N_mpc, r_drone, r_safety, x1_opt_old, x2_opt_old) #, v1_opt_old, v2_opt_old
    #         mpc_velocity_pub.publish(mpc_velocity)
 
    #         # We update this so that we can publish any target we want
    #         mpc_target_init.desired_position.x = actual_state.position.x
    #         mpc_target_init.desired_position.y = actual_state.position.y
    #         mpc_target_init.desired_position.z = actual_state.position.z

    #         x1_opt_old, x2_opt_old = x1_opt, x2_opt

    #     mpc_target_old = mpc_target
    #     rate.sleep()
    rate.sleep()

    rospy.spin()