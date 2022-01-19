from casadi import *
import numpy as np
import math
from enum import Enum
from crazy_common_py.common_functions import constrain, sameSign


class WindupType:
    Clamped = 0
    Exclusion = 1

class WindupInfo:
    def __init__(self, lowerValueSat=0, upperValueSat=0):
        self.lowerValueSat = lowerValueSat
        self.upperValueSat = upperValueSat

class PidController:
    def __init__(self, Kp, Ki, Kd, dt, windupType, integralLimit=0, windup_info=WindupInfo()):
        # Setting up the gains:
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd

        # Setting up time increment:
        self.dt = dt

        # Setting up windup check:
        self.windupType = windupType
        self.integralLimit = integralLimit
        self.windup_info = windup_info

        # Setting up the actual and past error value:
        self.error = 0
        self.prevError = 0
        self.deltaError = 0

        # Setting up contributions:
        self.proportional = 0
        self.integral = 0
        self.derivative = 0


    def updatePID(self, actual, desired, updateError=True):
        # Calculating actual error:
        '''
        e(t) = desiredValue - actualValue
        '''
        if updateError:
            self.error = desired - actual

        self.deltaError = self.error - self.prevError

        # Proportional contribution:
        '''
        ProportionalContribution = Kp * e(t)
        '''
        self.proportional = self.kp * self.error

        # Derivative contribution:
        '''
                                       e(t_{n}) - e(t_{n-1}) 
        DerivativeContribution = Kd * -----------------------
                                                dt
        '''
        self.derivative = self.kd * (self.deltaError / self.dt)

        # Integral contribution:
        '''
        IntegralContribution = Ki * \int_{0}^{t} e(t) * dt
        '''
        self.integral = self.integral + self.ki * (self.error * self.dt)

        # Preventing windup problem:
        if self.windupType == WindupType.Clamped:
            if self.integralLimit != 0:
                self.integral = constrain(self.integral, - self.integralLimit, self.integralLimit)
        elif self.windupType == WindupType.Exclusion:
            # Checking if saturation occured:
            beforeClamping = self.proportional + self.integral + self.derivative
            afterClamping = constrain(beforeClamping, self.windup_info.lowerValueSat, self.windup_info.upperValueSat)

            # Checking sign of the error and sign of the PID output:
            if beforeClamping != afterClamping and sameSign(beforeClamping, self.error):
                self.integral = 0

        # Updating previous error value:
        self.prevError = self.error

        return self.proportional + self.integral + self.derivative

    def reset(self):
        self.proportional = 0
        self.integral = 0
        self.derivative = 0

        self.error = 0
        self.prevError = 0


class MPCController:
    
    # Defining class variables
    r_drone = 0.05
    r_safety = 0.05
    g = 9.8065

    def __init__(self, x_obs, y_obs, r_obs, T_mpc, N_mpc):
        # Defining obstacle positions and radii as vectors
        self.x_obs = x_obs
        self.y_obs = y_obs
        self.r_obs = r_obs
        # Time horizon of the MPC Controller
        self.T_mpc = T_mpc
        # Number of control intervals
        self.N_mpc = N_mpc
        # Time step
        self.dt = self.T_mpc/self.N_mpc
    

    def updateMPC(self, actual_state, desired_position):
        # Setting the actual state, in particular position and velocity
        self.actual_state = actual_state
        self.x = self.actual_state.position.x
        self.y = self.actual_state.position.y
        self.z = self.actual_state.position.z
        self.vx = self.actual_state.velocity.x
        self.vy = self.actual_state.velocity.y
        self.vz = self.actual_state.velocity.z

        # Setting the desired state, in particular position and velocity
        self.desired_position = desired_position
        self.x_des = self.desired_position.position.x
        self.y_des = self.desired_position.position.y
        self.z_des = self.desired_position.position.z

        # Solving the NLP solver
        self.v_desired = self.nlp_solver()

        return self.v_desired


    def nlp_solver(self):

        # Declare model variables
        x1 = MX.sym('x1')
        x2 = MX.sym('x2')
        x3 = MX.sym('x3')
        x4 = MX.sym('x4')
        x5 = MX.sym('x5')
        x6 = MX.sym('x6')
        x = vertcat(x1, x2, x3, x4, x5, x6)


        u1 = MX.sym('u1')
        u2 = MX.sym('u2')
        u3 = MX.sym('u3')
        u = vertcat(u1, u2, u3)

        # Model equations
        xdot = vertcat(x4, x5, x6, u1, u2, u3 - self.g)

        # Objective term
        L = u1**2 + u2**2

        # Formulate discrete time dynamics
        if False:
            # CVODES from the SUNDIALS suite
            dae = {'x':x, 'p':u, 'ode':xdot, 'quad':L}
            opts = {'tf':T/N}
            F = integrator('F', 'cvodes', dae, opts)
        else:
            # Fixed step Runge-Kutta 4 integrator
            M = 4 # RK4 steps per interval
            DT = self.T_mpc/self.N_mpc/M
            f = Function('f', [x, u], [xdot, L])
            X0 = MX.sym('X0', 6)
            U = MX.sym('U', 3)
            X = X0
            Q = 0
            for j in range(M):
                k1, k1_q = f(X, U)
                k2, k2_q = f(X + DT/2 * k1, U)
                k3, k3_q = f(X + DT/2 * k2, U)
                k4, k4_q = f(X + DT * k3, U)
                X = X + DT/6*(k1 + 2*k2 + 2*k3 + k4)
                Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q)
            F = Function('F', [X0, U], [X, Q],['x0','p'],['xf','qf'])
        
        
        # MPC LOOP
        P_log = []
        px_log = []
        py_log = []
        U_log = []
        ux_log = []
        uy_log = []
        P_0 = [self.x, self.y, self.z, self.vx, self.vy, self.vz]
        P_N = [self.x_des, self.y_des, self.z_des, 0, 0, 0]
        u_ig = [0, 0, self.g]
        P_log = P_0 # initial state

        # Initializing state estimate at time instant i
        X_i = P_0
        u_i = u_ig
        # print('X_i: ', X_i)


        # Start with an empty NLP at each time step
        w = []
        # print('w: ', w)
        w0 = []
        lbw = []
        ubw = []
        J = 0
        g = []        # it includes the constraint for obstacle avoidance
        lbg = []
        ubg = []

        # "Lift" initial conditions
        Xk = MX.sym('X0', 6)
        w += [Xk]
        # print('X_i: ', X_i)
        # print('X_i[0]: ', X_i[0])

        lbw += [X_i[0].__float__(), X_i[1].__float__(), 
                X_i[2].__float__(), X_i[3].__float__(),
                X_i[4].__float__(), X_i[5].__float__()]
        ubw += [X_i[0].__float__(), X_i[1].__float__(), 
                X_i[2].__float__(), X_i[3].__float__(),
                X_i[4].__float__(), X_i[5].__float__()]
        w0 += [X_i[0].__float__(), X_i[1].__float__(), 
               X_i[2].__float__(), X_i[3].__float__(),
               X_i[4].__float__(), X_i[5].__float__()]

        # Formulate the NLP
        for k in range(self.N):
            # New NLP variable for the control
            Uk = MX.sym('U_' + str(k), 3)  # creating symbolic expression for the 
                                        # new optimization variable
            w   += [Uk]     
            lbw += [-20, -20, -20]
            ubw += [ 20,  20,  20]
            w0  += [u_i[0].__float__(), u_i[1].__float__(), u_i[2].__float__()]

            # Integrate till the end of the interval
            Fk = F(x0=Xk, p=Uk)         # we call the integrator
            Xk_end = Fk['xf']
            J=J+Fk['qf']

            # New NLP variable for state at end of interval
            Xk = MX.sym('X_' + str(k+1), 6)
            w   += [Xk]
            lbw += [-inf, -inf, -inf, -0.6, -0.6, -0.6]
            ubw += [ inf,  inf,  inf,  0.6,  0.6,  0.6]


            w0  += [X_i[0].__float__() + (k+1)/self.N*P_N[0], 
                    X_i[1].__float__() + (k+1)/self.N*P_N[1],
                    X_i[2].__float__() + (k+1)/self.N*P_N[2],
                    X_i[3].__float__() + (k+1)/self.N*P_N[3],
                    X_i[4].__float__() + (k+1)/self.N*P_N[4], 
                    X_i[5].__float__() + (k+1)/self.N*P_N[5]]

            # Add equality constraint (continuity constraint for multiple shooting)
            g   += [Xk_end-Xk]
            lbg += [0, 0, 0, 0, 0, 0]
            ubg += [0, 0, 0, 0, 0, 0]

            # Add inequality constraint for obstacle avoidance
            for ii in range(len(self.x_obs)): # start here
                g   += [(Xk[0] - self.x_obs[ii])**2 + (Xk[1] - self.y_obs[ii])**2]
                lbg += [(self.r_obs[ii] + self.r_drone + self.r_safety)**2]
                ubg += [+inf]


            if k == self.N_mpc - 1:
                X_final = P_N
                g += [Xk_end - X_final]
                lbg += [0, 0, 0, 0, 0, 0]
                ubg += [0, 0, 0, 0, 0, 0]

        prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}
        solver = nlpsol('solver', 'ipopt', prob);

        # Solve the NLP
        sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
        w_opt = sol['x'].full().flatten()

        # Extracting the optimal control inputs
        u1_opt = []
        u2_opt = []
        u3_opt = []
        u1_opt = w_opt[6::9]
        u2_opt = w_opt[7::9]
        u3_opt = w_opt[8::9]

        # Extracting the control inputs to be applied
        u_opt_i = []
        u_opt_i += [u1_opt[0]]
        u_opt_i += [u2_opt[0]]
        u_opt_i += [u3_opt[0]]
        self.u_i = np.array(u_opt_i)
        
        self.v_i = []
        self.v_i = [self.vx, self.vy, self.vz]
        self.v_i = np.array(self.v_i)

        self.v_desired = self.v_i + self.u_i*self.dt

        return self.v_desired

