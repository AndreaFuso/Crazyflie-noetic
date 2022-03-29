#! /usr/bin/env python3
# ROS MODULES
import rospy

# CUSTOM MODULES
from crazy_common_py.dataTypes import Vector3
from crazyflie_simulator.CrazySim import CrazySim
from std_msgs.msg import Empty, Int16
from crazyflie_messages.msg import Position, CrazyflieState, Attitude
import numpy as np
import cvxpy as cp      # cvxpy is a toolbox for convex optimization in python, 
                        # we need to solve a convex QP

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


        ########################################################################################

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



###########################################################################

#                S U B S C R I B E R     C A L L B A C K S

###########################################################################

# to initialize the desired velocity for the drone

def mpc2cbf_sub_callback(msg):
    mpc_velocity.desired_velocity.x = msg.desired_velocity.x
    mpc_velocity.desired_velocity.y = msg.desired_velocity.y
    # print('mpc_velocity is: ', mpc_velocity)

    cf_state.position.x = msg.desired_position.x
    cf_state.position.y = msg.desired_position.y
    print('cf_state is: ', cf_state)


    # sub_cbf_flag.data = 1

    print('starting cbf...')    
    #++++++++++++++ LOW LEVEL CBF CONTROLLER+++++++++++++++++++++++++++++++
    
    # Getting position of crazyflie
    P_0 = []
    P_0.append(cf_state.position.x)
    P_0.append(cf_state.position.y)

    print('P_0 is: ', P_0)

    # Extracting the mpc velocities from mpc_velocity
    v_mpc = []
    v_mpc.append(mpc_velocity.desired_velocity.x)
    v_mpc.append(mpc_velocity.desired_velocity.y)

    v_mpc = np.array(v_mpc)
    print('v_mpc is: ' , v_mpc)

    # Setting initial position at the current time step
    x0 = np.array(P_0)

    # Creating the instance of the CBF controller
    cbf_controller = CBF_controller(v_mpc, alpha, x0)

    # Setting obstacles
    cbf_controller.set_obstacle(N_obs, x_obs, r_obs)

    # Getting cbf velocity
    v = cbf_controller.get_cbf_v(x0)

    # Setting cbf_velocity msg to be published on /cf1/mpc_velocity
    mpc_velocity.desired_velocity.x = v[0]
    mpc_velocity.desired_velocity.y = v[1]

    print('mpc_velocity is: ', mpc_velocity)

    mpc_velocity_pub.publish(mpc_velocity)














if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('crazyflie_spawner_node', log_level=rospy.ERROR)

    # Extracting rosparam informations (to understand the name and spawn position):
    crazyflie_name = rospy.get_param('crazyflie_spawner_node/name')
    initial_pos = rospy.get_param('crazyflie_spawner_node/initial_position')

    # Spawning the virtual Crazyflie:
    cf = CrazySim(crazyflie_name, Vector3(initial_pos[0], initial_pos[1], initial_pos[2]))

    

    #++++++++++++++++++++++ CBF PARAMETERS +++++++++++++++++++++++++++++++++++++

    # Setting the parameters for the cbf controller and the position goal
    v_lim = 0.5
    alpha = 1.0
    d_lim = 0.4

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


    sub_cbf_flag = Int16()
    sub_cbf_flag.data = 0

    ###############################################################################

    #                     P U B L I S H E R S   S E T U P

    ###############################################################################

    # Publisher to publish the target velocity (output of cbf controller)
    mpc_velocity_pub = rospy.Publisher('/' + crazyflie_name + '/mpc_velocity', 
                                            Position, queue_size=1)
    mpc_velocity = Position()

    mpc_velocity.desired_velocity.x = 0
    mpc_velocity.desired_velocity.y = 0

    ###############################################################################

    #                     S U B S C R I B E R S   S E T U P

    ###############################################################################

    # Subscriber to get the mpc target position
    mpc_velocity = Position()
    mpc2cbf_sub = rospy.Subscriber('/' + crazyflie_name + '/mpc2cbf_velocity', 
                                   Position, mpc2cbf_sub_callback)
    


    # Subscriber to get the state
    cf_state = CrazyflieState()
    # state_sub = rospy.Subscriber('/' + crazyflie_name + '/state', 
    #                                CrazyflieState, state_sub_callback)
    

    rospy.spin()

