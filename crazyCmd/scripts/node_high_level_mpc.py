#! /usr/bin/env python3
import rospy
import time
from casadi import *
import numpy as np
from crazyflie_simulator.CrazySim import CrazySim
from crazyflie_manager.CrazyManager import *
from crazy_common_py.dataTypes import Vector3
from crazy_common_py.common_functions import rad2deg, deg2rad
from std_msgs.msg import Empty, Int16
from crazyflie_messages.msg import Position, CrazyflieState, Attitude
from crazy_common_py.common_functions import standardNameList
from crazyflie_swarm.CrazySwarmSim import CrazySwarmSim


###################################################################

#                 H I G H    L E V E L    M P C       

###################################################################


def nlp_solver_2d(N_cf, P_N, P_0, T, N, x_opt, v_opt, 
                  d_ref, d_neigh, v_ref, w_sep, w_nav, w_dir, N_mid):

    # Calculating the reference direction for the drone in the middle
    u_ref = [P_N[N_mid] - P_0[N_mid], P_N[N_mid+1] - P_0[N_mid+1]]
    u_ref = np.array(u_ref)
    u_ref = u_ref/np.linalg.norm(u_ref)
    u_refx = u_ref[0]
    u_refy = u_ref[1]

    x_pos = P_0[N_mid]
    y_pos = P_0[N_mid+1]

    x_des = P_N[N_mid]
    y_des = P_N[N_mid]


    # Setting desired velocity
    vx_des = (x_des-x_pos)/T
    vy_des = (y_des-y_pos)/T
    if vx_des > 1:
        vx_des = 1
    if vy_des > 1:
        vy_des = 1

    
    # Declare model variables
    x = MX.sym('x', N_cf*2)

    v = MX.sym('v', N_cf*2)

    # Model equations
    xdot = v

    # Objective term
    a_sep = 2


    list_neighbours_i = range(N_cf)

    L = 0

    for ii in range(N_cf):
        for kk in list_neighbours_i:
            # Separation cost
            if ii != kk:
                L += w_sep*((x[ii*2]-x[kk*2])**2 \
                    + (x[ii*2+1]-x[kk*2+1])**2\
                    - d_ref**2)
        
        # Navigation cost
        L += w_nav*(v[ii*2]**2 + v[ii*2+1]**2 - v_ref**2)

        # Direction cost
        L += w_dir*(v[ii*2]**2 + v[ii*2+1]**2 - \
            (v[ii*2]*u_refx + v[ii*2+1]*u_refy)**2)


    # Formulate discrete time dynamics
    if False:
       # CVODES from the SUNDIALS suite
       dae = {'x':x, 'p':v, 'ode':xdot, 'quad':L}
       opts = {'tf':T/N}
       F = integrator('F', 'cvodes', dae, opts)
    else:
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


        ################ Final target on line ##########################

        if k == N - 1:
            X_final = P_N
            g += [Xk_end - X_final]

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

        ################################################################

        ################ Final target on circumference #################

        # if k == N - 1:
        #     x_final = 2
        #     y_final = 1
        #     r_circ = 0.2
        #     for ii in range(N_cf):
        #         g += [(Xk[2*ii] - x_final)**2 + (Xk[2*ii+1] - y_final)**2 - r_circ**2]

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
    prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}
    solver = nlpsol('solver', 'ipopt', prob);

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

    # Some constants
    d_ref = 0.3 # reference distance between agents
    d_neigh = 0.8 # neighbour distance
    v_ref = 0.5 # reference velocity

    # Weights for objective function
    w_sep = 0.01*number_of_cfs**-1.7
    w_nav = 1
    w_dir = 1

    # Number of the drone in the middle
    N_mid = int(number_of_cfs/2)
    if N_mid % 2 != 0:
        N_mid = int((number_of_cfs-1)/2)


    ###############################################################################

    #                     P U B L I S H E R S   S E T U P

    ###############################################################################

    # Publisher to publish the target velocity (output of nlp)

    # List of velocities used to collect the output of the nlp solver
    mpc_velocity = []
    for ii in range(number_of_cfs):
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


    rate = rospy.Rate(10)

    # rate = rospy.Rate(N_mpc/T_mpc)

    while not rospy.is_shutdown():

        # Initializing the initial position of agents at each step
        P_0 = []
        for ii in range(number_of_cfs):
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
            for ii in range(number_of_cfs):
                P_N.append(mpc_target.desired_position.x + (ii - N_mid)*d_ref)
                P_N.append(mpc_target.desired_position.y)            

            # Initializing the optimal velocity of agents to use it 
            # for the hot start initial guess
            v_opt_old = []
            for ii in range(2*number_of_cfs):
                v_opt_old.append(np.linspace(0, 0, N_mpc+1))

            # Initializing optimal positions to use them as initial guess 
            # for the hot start initial guess
            x_opt_old = []
            for ii in range(2*number_of_cfs):
                x_opt_old.append(np.linspace(P_0[ii], P_N[ii], N_mpc+1))

            sub_mpc_flag.data = 2

        else:
            # Once the flag is set to 2, the nlp solver is called at each iteration
            # until a new mpc target is set and the 
            mpc_velocity, x_opt, v_opt = nlp_solver_2d(number_of_cfs, P_N, P_0, T_mpc, 
                                                       N_mpc, x_opt_old, v_opt_old,
                                                       d_ref, d_neigh, v_ref,
                                                       w_sep, w_nav, w_dir, N_mid)
            
            swarm_mpc_velocity_pub(mpc_velocity)

            x_opt_old, v_opt_old = x_opt, v_opt

        rate.sleep()

