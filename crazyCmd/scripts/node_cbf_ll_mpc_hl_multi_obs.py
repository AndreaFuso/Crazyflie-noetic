#! /usr/bin/env python3
from math import floor
import rospy
import time
from casadi import *
import numpy as np
import cvxpy as cp      # cvxpy is a toolbox for convex optimization in python, 
                        # we need to solve a convex QP
from crazyflie_simulator.CrazySim import CrazySim
from crazyflie_manager.CrazyManager import *
from crazy_common_py.dataTypes import Vector3
from crazy_common_py.common_functions import rad2deg, deg2rad
from std_msgs.msg import Empty, Int16
from crazyflie_messages.msg import Position, CrazyflieState, Attitude
from crazy_common_py.common_functions import standardNameList
from crazyflie_swarm.CrazySwarmSim import CrazySwarmSim






###################################################################

#      L O W     L E V E L    C B F     C O N T R O L L E R      

###################################################################

class CBF_controller():

    def __init__(self, v_mpc, alpha, x):
        
        self.x = x
        self.v_mpc = v_mpc
        self.alpha  = alpha

    def set_obstacle(self, N_obs, x_obs, r_obs):
        
       
        ########################################################################################


        # for ii in range(N_obs):

        # Defining the h function for the known obstacles
        self.h = lambda x1,x2,x_o,r_o : ((x1-x_o[0])**2 + (x2-x_o[1])**2 
                                    - r_o**2)*0.5

        self.grad_h = lambda x1,x2,x_o,r_o : np.array([x1-x_o[0] , x2-x_o[1]])


        ########################################################################################

        # self.h1 = lambda x1,x2 : ((x1-x_obs[0][0])**2 + (x2-x_obs[0][1])**2 \
        #                                 - r_obs[0]**2)*0.5
        # self.grad_h1 = lambda x1,x2 : np.array([x1-x_obs[0][0] , x2-x_obs[0][1]])


        # self.h2 = lambda x1,x2 : ((x1-x_obs[1][0])**2 + (x2-x_obs[1][1])**2 
        #                                 - r_obs[1]**2)*0.5
        # self.grad_h2 = lambda x1,x2 : np.array([x1-x_obs[1][0] , x2-x_obs[1][1]])


        # self.h3 = lambda x1,x2 : ((x1-x_obs[2][0])**2 + (x2-x_obs[2][1])**2 
        #                                 - r_obs[2]**2)*0.5
        # self.grad_h3 = lambda x1,x2 : np.array([x1-x_obs[2][0] , x2-x_obs[2][1]])




    def get_cbf_v(self, x):

        # Given current state solve pointwise Quadratic Program and get safe 
        # control action (velocity)

        v_opt      = cp.Variable(2)


        ########################################################################################
        
        h_num = []
        grad_h_num = []

        for ii in range(N_obs):
            h_num.append(self.h(x[0], x[1], x_obs[ii], r_obs[ii])) 
            grad_h_num.append(self.grad_h(x[0], x[1], x_obs[ii], r_obs[ii]))

        ########################################################################################




        # h_num1 = self.h1(x[0],x[1])
        # grad_h_num1 = self.grad_h1(x[0],x[1])

        # h_num2 = self.h2(x[0],x[1])
        # grad_h_num2 = self.grad_h2(x[0],x[1])


        # h_num3 = self.h3(x[0],x[1])
        # grad_h_num3 = self.grad_h3(x[0],x[1])


        ########################################################################################


        v_des      = self.v_mpc

        # Solve QP for u_opt
        obj         = cp.Minimize((1/2)*cp.quad_form(v_opt-v_des,np.eye(2)))


        ########################################################################################


        constraints = []

        for ii in range(N_obs):
            obs_distance = np.linalg.norm(np.array([x[0]-x_obs[ii][0],
                                                    x[1]-x_obs[ii][1]]))
            if obs_distance < (r_obs[ii]+1.5):
                constraints.append(grad_h_num[ii].T @ v_opt >= -self.alpha*h_num[ii])


        ########################################################################################

        # constraints = [grad_h_num1.T @ v_opt >= -self.alpha*h_num1,
        #                grad_h_num2.T @ v_opt >= -self.alpha*h_num2,
        #                grad_h_num3.T @ v_opt >= -self.alpha*h_num3]

        ########################################################################################



        prob = cp.Problem(obj,constraints)
        prob.solve()

        return v_opt.value




###################################################################

#                 H I G H    L E V E L    M P C       

###################################################################


def nlp_solver_2d(N_cf, P_N, P_0, N, x_opt, v_opt, 
                  d_ref, d_neigh, v_ref, w_sep, 
                  w_final, w_vel, N_neigh):

    # Getting center of mass of the swarm
    x_cm = np.array(P_0[0::2]).sum()/N_cf
    y_cm = np.array(P_0[1::2]).sum()/N_cf

    # Desired posittions
    x_des = P_N[0]
    y_des = P_N[1]

    # Calculating the reference direction for the drone swarm    
    u_ref = [x_des-x_cm, y_des-y_cm]
    u_ref = np.array(u_ref)
    u_norm = np.linalg.norm(u_ref)

    # Calculating the time horizon for the MPC
    T = u_norm/v_ref

    # Threshold on time horizon, otherwise the swarm oscillates
    if T < 1.0:
        T = 1.0



    # Versor for the reference direction
    u_ref = u_ref/u_norm

    u_refx = u_ref[0]
    u_refy = u_ref[1]

    # Setting desired velocity
    vx_des = (x_des-x_cm)/T
    vy_des = (y_des-y_cm)/T

    # Saturation on desired velocity
    if vx_des > 1:
        vx_des = 1
    if vx_des < -1:
        vx_des = -1

    if vy_des > 1:
        vy_des = 1
    if vy_des < -1:
        vy_des = -1
    
    # Declare model variables
    x = MX.sym('x', N_cf*2)

    v = MX.sym('v', N_cf*2)

    # Model equations
    xdot = v



    ##############################################################################

    # # Building Adjacency Matrix
    # A_neigh = np.zeros((N_cf, N_cf))
    
    # index_neigh = []

    # for ii in range(N_cf-1):
    #     list_p_rel = []
    #     for jj in range(ii + 1, N_cf):
    #         p_rel = [P_0[2*ii] - P_0[2*jj], P_0[2*ii+1] - P_0[2*jj+1]]
    #         p_rel = np.array(p_rel)
    #         # Storing p_rel in a list
    #         list_p_rel.append(np.linalg.norm(p_rel))

    #         # Filling Adjacency Matrix
    #         if np.linalg.norm(p_rel) < d_neigh:
    #             A_neigh[ii,jj], A_neigh[jj,ii] = 1, 1
        


    #     # Getting the sorted indices of list_p_rel to set the order of neighbours
    #     index_neigh.append((np.argsort(list_p_rel)+ii+1).tolist())
    # print('index_neigh is: ', index_neigh)
    # print('A_neigh is: ', A_neigh)


    # # list_neighbours_i = range(N_cf)



    # ###################### Ordered list of neighbours ############################
    # list_list_neighbours = []
    
    
    # for ii in range(N_cf-1):

    #     neigh_count = 0

    #     list_neighbours_i = []

    #     for jj in range(N_cf):
    #         if A_neigh[ii,jj] == 1 and jj > ii:
    #             neigh_count += 1

    #     list_neighbours_i = index_neigh[ii][:neigh_count]
    #     print('list_neighbours_i is: ', list_neighbours_i)

    #     list_list_neighbours.append(list_neighbours_i)

    # # list_neighbours_i = [list_neighbours_i for ii in index_neigh[jj]]
    # print('list_list_neighbours is: ', list_list_neighbours)
    # # list_neighbours_i = [x for _, x in sorted(list_neighbours_i,)]

    # ############## Weights for the distances in separation cost ##################
    # list_list_weights = []
    # for ii in range(N_cf-1):
    #     list_weights_i = []
    #     ll = len(list_list_neighbours[ii])
    #     for jj in range(ll):
    #         kk = floor(jj/2)
    #         list_weights_i.append(kk)
    #     list_list_weights.append(list_weights_i)

    # print('list_list_weights is: ', list_list_weights)


    ##############################################################################

    # Building Ordered list of 3 closest neighbors for each drone
    
    index_neigh = []

    for ii in range(N_cf):
        list_p_rel = []
        for jj in range(N_cf):
            p_rel = [P_0[2*ii] - P_0[2*jj], P_0[2*ii+1] - P_0[2*jj+1]]
            p_rel = np.array(p_rel)
            # Storing p_rel in a list
            list_p_rel.append(np.linalg.norm(p_rel))
        print('list_p_rel is: ', list_p_rel)

        # Getting the sorted indices of list_p_rel to set the order of neighbours
        index_neigh_i = (np.argsort(list_p_rel)).tolist()
        index_neigh_i.pop()
        index_neigh.append(index_neigh_i)
    print('index_neigh is: ', index_neigh)


    ###################### Ordered list of neighbours ############################
    list_list_neighbours = []
    
    
    for ii in range(N_cf):
        if N_cf > N_neigh:
            list_neighbours_i = index_neigh[ii][:N_neigh]
        else:
            list_neighbours_i = index_neigh[ii][:(N_cf-1)]
        list_list_neighbours.append(list_neighbours_i)
        print('list_neighbours_i is: ', list_neighbours_i)

    
    print('list_list_neighbours is: ', list_list_neighbours)




    ##############################################################################

    # Building Objective Function

    L = 0
    x_cm = 0
    y_cm = 0

    # # 1 All neighbours
    # for ii in range(N_cf-1):
    #     for kk in range(ii+1, N_cf):
    #         # if ii != kk:
    #         # Separation cost
    #         n_neigh = A_neigh[ii].sum()
            
    #         d_ref_sep = d_ref #*(1 + 0.2*n_neigh)
    #         L += w_sep*((x[ii*2]-x[kk*2])**2 \
    #             + (x[ii*2+1]-x[kk*2+1])**2\
    #             - d_ref_sep**2)**2
    #     print('n_neigh is: ', n_neigh)


    # # 2
    # for ii in range(N_cf-1):
    #     jj = 0
    #     for kk in list_list_neighbours[ii]:
    #         jj += 1
    #         # Separation cost
    #         d_ref_sep = d_ref*(1 + list_list_weights[ii][jj-1]) 
    #         L += w_sep*((x[ii*2]-x[kk*2])**2 \
    #             + (x[ii*2+1]-x[kk*2+1])**2\
    #             - d_ref_sep**2)**2



    # 3
    for ii in range(N_cf):
        for kk in list_list_neighbours[ii]:
            # Separation cost
            d_ref_sep = d_ref
            L += w_sep*((x[ii*2]-x[kk*2])**2 \
                + (x[ii*2+1]-x[kk*2+1])**2\
                - d_ref_sep**2)**2

    for ii in range(N_cf):

        # # Navigation cost
        # L += w_nav*(v[ii*2]**2 + v[ii*2+1]**2 - v_ref_new**2)**2

        # # Conditional final position cost
        # p_rel_final = [x_des - P_0[2*ii], y_des - P_0[2*ii+1]]
        # p_rel_final = np.array(p_rel_final)
        # d_final = np.linalg.norm(p_rel_final)

        # if d_final > 1.5:
        #     L += w_final*((x[ii*2] - x_des)**2 + (x[ii*2+1] - y_des)**2)


        # Final Position cost
        L += w_final*((x[ii*2] - x_des)**2 + (x[ii*2+1] - y_des)**2)

        # Control input cost
        L += w_vel*(v[ii*2]**2 + v[ii*2+1]**2)

        # # Direction cost
        # L += w_dir*(v[ii*2]**2 + v[ii*2+1]**2 - \
        #     (v[ii*2]*u_refx + v[ii*2+1]*u_refy)**2)**2

        x_cm += x[ii*2]
        y_cm += x[ii*2+1]
    
    x_cm = x_cm/N_cf
    y_cm = y_cm/N_cf





    # for ii in range(N_cf):

    #     # Control input cost

    #     if abs(P_0[2*ii] - x_des) < 1 and abs(P_0[2*ii+1] - y_des) < 1: 
    #         w_vel_i = 10*w_vel
    #     else:
    #         w_vel_i = w_vel

    #     L += w_vel_i*(v[ii*2]**2 + v[ii*2+1]**2)

    # # Final Position cost for center of mass
    # L += w_final*(P_N[0] - x_cm)**2
    # L += w_final*(P_N[1] - y_cm)**2

    # Formulate discrete time dynamics
    if False:
        # CVODES from the SUNDIALS suite
        dae = {'x':x, 'p':v, 'ode':xdot, 'quad':L}
        opts = {'tf':T/N}
        F = integrator('F', 'cvodes', dae, opts)
    
    elif False:
        # Fixed step Runge-Kutta 4 integrator
        M = 4 # RK4 steps per interval
        DT = T/N/M
        f = Function('f', [x, v], [xdot, L])
        X0 = MX.sym('X0', 2*N_cf)
        U = MX.sym('U', 2*N_cf)
        X = X0
        Q = 0
        for j in range(M):
            k1, k1_q = f(X, U)
            k2, k2_q = f(X + DT/2 * k1, U)
            k3, k3_q = f(X + DT/2 * k2, U)
            k4, k4_q = f(X + DT * k3, U)
            X = X + DT/6*(k1 + 2*k2 + 2*k3 + k4)
            Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q)
        F = Function('F', [X0, U], [X, Q], ['x0','p'], ['xf','qf'])
    
    else:
        # Forward Euler
        DT = T/N
        f = Function('f', [x, v], [xdot, L])
        X0 = MX.sym('X0', 2*N_cf)
        U = MX.sym('U', 2*N_cf)
        X = X0
        Q = 0
        k, k_q = f(X,U)
        X = X + k*DT
        Q = Q + k_q*DT
        F  = Function('F', [X0, U], [X, Q], ['x0','p'], ['xf','qf'])

    # Evaluate at a test point
    # Fk = F(x0=[0.2, 0.3],p=[0.4, 2, 9])
    # print(Fk['xf'])
    # print(Fk['qf'])


    # -------------------------------------------------------------------

    #           S E T T I N G   U P   T H E   M P C

    # -------------------------------------------------------------------
    '''
    We only exploit the first control input within the MPC framework.
    Hence, we define a casadi function which allows us to get the 
    control input at the first time step as output, given the initial state
    as input.
    '''

    # MPC LOOP
    P_log = []
    px_log = []
    py_log = []
    V_log = []
    ux_log = []
    uy_log = []

    v_ig = []
    for ii in range(N_cf):
        v_ig.append(vx_des)
        v_ig.append(vy_des)
    P_log = P_0 # initial state

    # Initializing state estimate at time instant i
    X_i = P_0
    v_i = v_ig


    # Start with an empty NLP at each time step
    w = []
    w0 = []
    lbw = []
    ubw = []
    J = 0
    g = []
    lbg = []
    ubg = []

    # "Lift" initial conditions
    Xk = MX.sym('X0', 2*N_cf)
    w += [Xk]

    lbw_k = []
    for ii in range(N_cf):
        lbw_k.append(X_i[2*ii].__float__())
        lbw_k.append(X_i[2*ii+1].__float__())

    lbw += lbw_k

    ubw_k = []
    for ii in range(N_cf):
        ubw_k.append(X_i[2*ii].__float__())
        ubw_k.append(X_i[2*ii+1].__float__())

    ubw += ubw_k

    w0_k = []
    for ii in range(N_cf):
        w0_k.append(X_i[2*ii].__float__())
        w0_k.append(X_i[2*ii+1].__float__())

    w0 += w0_k


    # Formulate the NLP
    for k in range(N):
        # New NLP variable for the control
        Vk = MX.sym('V_' + str(k), 2*N_cf)  # creating symbolic expression for the 
                                            # new optimization variable
        w   += [Vk]

        lbw_k = []
        for ii in range(N_cf):
            lbw_k.append(-inf)
            lbw_k.append(-inf)

        lbw += lbw_k

        ubw_k = []
        for ii in range(N_cf):
            ubw_k.append(+inf)
            ubw_k.append(+inf)

        ubw += ubw_k

        w0_k = []
        for ii in range(N_cf):
            w0_k.append(v_opt[2*ii][k].__float__())
            w0_k.append(v_opt[2*ii+1][k].__float__())

        w0 += w0_k


        # Integrate till the end of the interval
        Fk = F(x0=Xk, p=Vk)         # we call the integrator
        Xk_end = Fk['xf']
        J=J+Fk['qf']

        # New NLP variable for state at end of interval
        Xk = MX.sym('X_' + str(k+1), 2*N_cf)
        w   += [Xk]


        lbw_k = []
        for ii in range(N_cf):
            lbw_k.append(-inf)
            lbw_k.append(-inf)
        
        lbw += lbw_k

        ubw_k = []
        for ii in range(N_cf):
            ubw_k.append(+inf)
            ubw_k.append(+inf)

        ubw += ubw_k

        w0_k = []
        for ii in range(N_cf):
            w0_k.append(x_opt[2*ii][k+1].__float__())
            w0_k.append(x_opt[2*ii+1][k+1].__float__())

        w0 += w0_k


        # Add equality constraint (continuity constraint for multiple shooting)
        g   += [Xk_end-Xk]

        lbg_k = []
        for ii in range(N_cf):
            lbg_k.append(0)
            lbg_k.append(0)

        lbg += lbg_k

        ubg_k = []
        for ii in range(N_cf):
            ubg_k.append(0)
            ubg_k.append(0)

        ubg += ubg_k


        # # Add inequality constraint for obstacle avoidance between drones
        # # if k > 1:
        # # if k > 0:
        # for ii in range(N_cf):
        #     for jj in list_list_neighbours[ii]:
        #         g   += [(Xk_end[2*ii] - Xk_end[2*jj])**2 + 
        #                 (Xk_end[2*ii+1] - Xk_end[2*jj+1])**2]
        #         lbg += [(6*r_drone)**2]
        #         ubg += [+inf]


        # Add inequality constraint for obstacle avoidance between drones
        # if k > 1:
        # if k > 0:
        for ii in range(N_cf-1):
            for jj in range(ii+1,N_cf):
                g   += [(Xk_end[2*ii] - Xk_end[2*jj])**2 + 
                        (Xk_end[2*ii+1] - Xk_end[2*jj+1])**2]
                lbg += [(8*r_drone)**2]
                ubg += [+inf]


        ################ Center of mass in final target ##########################

        if k == N - 1:

            x_cm_end = 0
            y_cm_end = 0

            for ii in range(N_cf):
                x_cm_end += Xk_end[2*ii]
                y_cm_end += Xk_end[2*ii+1]

            x_cm_end = x_cm_end/N_cf
            y_cm_end = y_cm_end/N_cf

            g += [P_N[0] - x_cm_end, P_N[1] - y_cm_end]

            lbg += [0, 0]
            ubg += [0, 0]

        ################################################################

        ########## Leader on target ####################################

        # if k == N - 1:
        #     g += [P_N[0] - Xk_end[0], P_N[1] - Xk_end[1]]
        #     lbg += [0, 0]
        #     ubg += [0, 0]

        ################ Final target on line ##########################

        # if k == N - 1:
        #     X_final = P_N
        #     g += [Xk_end - X_final]

        #     lbg_k = []

        #     for ii in range(N_cf):
        #         lbg_k.append(0)
        #         lbg_k.append(0)

        #     lbg += lbg_k

        #     ubg_k = []

        #     for ii in range(N_cf):
        #         ubg_k.append(0)
        #         ubg_k.append(0)

        #     ubg += ubg_k

        ################################################################


        ################ Final target on circumference #################

        # if k == N - 1:
        #     x_final = 2
        #     y_final = 1
        #     r_circ = 0.3
        #     for ii in range(N_cf):
        #         g += [(Xk[2*ii] - P_N[0])**2 + \
        #               (Xk[2*ii+1] - P_N[1])**2 - r_circ**2]

        #     lbg_k = []
        #     for ii in range(N_cf):
        #         lbg_k.append(0)

        #     lbg += lbg_k

        #     ubg_k = []
        #     for ii in range(N_cf):
        #         ubg_k.append(0)

        #     ubg += ubg_k

    ####################################################################


    # Create an NLP solver
    # NLP solver options
    opts = {}
    opts["max_iter_eig"] = 5
    
    prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}
    solver = nlpsol('solver', 'ipopt', prob, opts);

    # Solve the NLP
    sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
    w_opt = sol['x'].full().flatten()


    # Extracting the optimal state
    x_opt = []
    for ii in range(2*N_cf):
        x_opt.append(w_opt[ii::4*N_cf])

    # Extracting the optimal control input
    v_opt = []
    for ii in range(2*N_cf):
        v_opt.append(w_opt[(ii+2*N_cf)::4*N_cf])


    # Extracting only the first optimal control input
    v_opt_i = []
    for ii in range(N_cf):
        v_opt_i.append(v_opt[2*ii][0])
        v_opt_i.append(v_opt[2*ii+1][0])


    v_i = v_opt_i

    mpc_velocity = []

    for ii in range(N_cf):
        mpc_velocity.append(Position())
    for ii in range(N_cf):
        mpc_velocity[ii].desired_velocity.x = v_i[2*ii]
        mpc_velocity[ii].desired_velocity.y = v_i[2*ii+1]
    
    return mpc_velocity, x_opt, v_opt

###########################################################################

################## to be implemented in ROS ###############################

# # we need the actual measurement to build the complete list of neighbours

# # matrix_neighbours = []
# # list_neighbours_i = []

# # for ii in range(N_cf):
# #     for kk in range(N_cf):
# #         if ((x[ii*2]-x[kk*2])**2 + (x[ii*2+1]-x[kk*2+1])**2) < d_neigh**2 \
# #               and ii != kk:
# #             list_neighbours_i.append(kk)
# #     matrix_neighbours.append(list_neighbours_i)

###########################################################################



###########################################################################

#                S U B S C R I B E R     C A L L B A C K S

###########################################################################

def mpc_target_sub_callback(msg):
    mpc_target.desired_position.x = msg.desired_position.x
    mpc_target.desired_position.y = msg.desired_position.y
    mpc_target.desired_position.z = msg.desired_position.z
    sub_mpc_flag.data = 1


###########################################################################

#                P U B L I S H E R      F U N C T I O N S

###########################################################################

def make_mpc_velocity_publishers():
    for cf_name in cf_names:
        mpc_velocity_pub = rospy.Publisher('/' + cf_name + '/mpc_velocity', 
                                            Position, queue_size=1)
        mpc_velocity_publishers.append(mpc_velocity_pub)


def swarm_mpc_velocity_pub(mpc_velocity):
    index = 0
    for index, mpc_velocity_pub in enumerate(mpc_velocity_publishers):
        mpc_velocity_pub.publish(mpc_velocity[index])

###########################################################################

#                                M A I N

###########################################################################


if __name__ == '__main__':
    
    # Node initialization:
    rospy.init_node('node_high_level_mpc', log_level=rospy.DEBUG)

    # Extracting rosparam informations (to understand the number of crazyflies):
    N_cf = rospy.get_param('swarm_node/cfs_number')
    print(N_cf)

    # Generate a standard list of names:
    cf_names = standardNameList(N_cf)
    print(cf_names)

    # Instantiate a swarm:
    swarm = CrazySwarmSim(cf_names)


    #++++++++++++++++++++++ MPC PARAMETERS +++++++++++++++++++++++++++++++++++++

    # Time interval and number of control intervals for the MPC
    T_mpc = 5
    # N_mpc = 10
    N_mpc = 5
    # Some constants
    d_neigh = 1.5 # + 1.25*number_of_cfs # neighbour distance
    d_ref = 1 #+ 0.1*number_of_cfs #+ 0.05*number_of_cfs # 0.15*number_of_cfs # reference distance between agents
    
    v_ref = 0.5 # reference velocity
    d_final_lim = 0.01

    N_neigh = 2

    # Weights for objective function
    w_sep = 4 #0.01*number_of_cfs**-1
    w_nav = 10 #100
    w_dir = 1
    w_final = 4
    w_vel = 100

    #++++++++++++++++++++++ CBF PARAMETERS +++++++++++++++++++++++++++++++++++++

    # Setting the parameters for the cbf controller and the position goal
    v_lim = 0.5
    alpha = 1.0
    d_lim = 0.4

    # # Defining obstacle geometry
    # x_obs = np.array([2.0, 2.5])
    # r_obs = 0.30
    # r_drone = 0.07
    # r_safety = 0.10
    # r_tot = r_obs + r_drone + r_safety
    
    # Defining obstacles geometry

    N_obs = 3

    x_obs = []

    x_obs_1 = np.array([2.0, 2.5])
    x_obs.append(x_obs_1)
    x_obs_2 = np.array([1.0, 2.5])
    x_obs.append(x_obs_2)
    x_obs_3 = np.array([2.0, 4.0])
    x_obs.append(x_obs_3)
    print('x_obs is: ', x_obs)
    


    r_drone = 0.07
    r_safety = 0.10

    r_obs = []

    r_obs_1 = 0.30 + r_drone + r_safety
    r_obs.append(r_obs_1)
    r_obs_2 = 0.20 + r_drone + r_safety
    r_obs.append(r_obs_2)
    r_obs_3 = 0.40 + r_drone + r_safety
    r_obs.append(r_obs_3)
    print('r_obs is: ', r_obs)


    ###############################################################################

    #                     P U B L I S H E R S   S E T U P

    ###############################################################################

    # Publisher to publish the target velocity (output of nlp)

    # List of velocities used to collect the output of the nlp solver
    mpc_velocity = []
    for ii in range(N_cf):
        mpc_velocity.append(Position())

    # List of mpc_velocity Publishers
    mpc_velocity_publishers = []
    make_mpc_velocity_publishers()


    ###############################################################################

    #                     S U B S C R I B E R S   S E T U P

    ###############################################################################

    # Subscriber to get the mpc target position
    mpc_target_sub = rospy.Subscriber('/swarm/mpc_target', Position, mpc_target_sub_callback)
    mpc_target = Position()

    ###############################################################################
    # Flag for the mpc target subscriber
    sub_mpc_flag = Int16()
    sub_mpc_flag.data = 0

    # Initializing mpc_target, mpc_target_init and mpc_target_old
    mpc_target.desired_position.x = 0
    mpc_target.desired_position.y = 0


    rate = rospy.Rate(20)

    # rate = rospy.Rate(100)

    # rate = rospy.Rate(N_mpc/T_mpc)

    while not rospy.is_shutdown():

        # Initializing the initial position of agents at each step
        P_0 = []
        for ii in range(N_cf):
            P_0.append(swarm.states[ii].position.x)
            P_0.append(swarm.states[ii].position.y)

        # print('P_0 is: ', P_0)
        
        if sub_mpc_flag.data == 0:
            # nothing is executed if no mpc target has been published
            pass

        elif sub_mpc_flag.data == 1:  # in case a new target is set
            
            # Initializing the target position for each drone starting from
            # mpc target
            P_N = []
            for ii in range(N_cf):
                P_N.append(mpc_target.desired_position.x)
                P_N.append(mpc_target.desired_position.y)            
            print('P_N is: ', P_N)
            # Initializing the optimal velocity of agents to use it 
            # for the hot start initial guess
            v_opt_old = []
            for ii in range(2*N_cf):
                v_opt_old.append(np.linspace(0, 0, N_mpc+1))

            # Initializing optimal positions to use them as initial guess 
            # for the hot start initial guess
            x_opt_old = []
            for ii in range(2*N_cf):
                x_opt_old.append(np.linspace(P_0[ii], P_N[ii], N_mpc+1))

            sub_mpc_flag.data = 2

        else:
            # Once the flag is set to 2, the nlp solver is called at each iteration
            # until a new mpc target is set and the 
            
            #++++++++++++++ HIGH LEVEL MPC CONTROLLER+++++++++++++++++++++++++++++++

            mpc_velocity, x_opt, v_opt = nlp_solver_2d(N_cf, P_N, P_0, 
                                                       N_mpc, x_opt_old, v_opt_old,
                                                       d_ref, d_neigh, v_ref,
                                                       w_sep, w_final, w_vel, N_neigh)
            
            x_opt_old, v_opt_old = x_opt, v_opt
            
            
            #++++++++++++++ LOW LEVEL CBF CONTROLLER+++++++++++++++++++++++++++++++
                        
            # Getting the new goal
            x_goal = np.array([mpc_target.desired_position.x, mpc_target.desired_position.y])

            for ii in range(N_cf):
                # Extracting the mpc velocities from mpc_velocity list
                v_mpc_x = mpc_velocity[ii].desired_velocity.x
                v_mpc_y = mpc_velocity[ii].desired_velocity.y
                v_mpc = np.array([v_mpc_x, v_mpc_y])

                # Setting initial position at the current time step
                x0 = np.array([P_0[2*ii], P_0[2*ii+1]])
                # Creating the instance of the CBF controller
                cbf_controller = CBF_controller(v_mpc, alpha, x0)
                # Setting obstacles
                cbf_controller.set_obstacle(N_obs, x_obs, r_obs)
                # Getting cbf velocity
                v = cbf_controller.get_cbf_v(x0)
                # Setting cbf_velocity msg to be published on /cf1/mpc_velocity
                mpc_velocity[ii].desired_velocity.x = v[0]
                mpc_velocity[ii].desired_velocity.y = v[1]
            
                        
            swarm_mpc_velocity_pub(mpc_velocity)


        rate.sleep()

