#! /usr/bin/env python3

from cmath import sqrt
from tkinter import N
import rosbag
import rospkg
import matplotlib.pyplot as plt
import numpy as np
import scipy as sp
from scipy import integrate


from crazy_common_py.common_functions import rad2deg


N_cf = 1
N_obs = 1
N_cbf_real = 5
mpc_traj_flag = True

rospack = rospkg.RosPack()

# +++++++++++++++++++++ Extracting data about cbf sim +++++++++++++++++++++++++++++++++

bag_name_cbf_sim = '1_drone_cbf_sim_test15.bag'
bag_path_cbf_sim = rospack.get_path('crazyCmd') + '/data/output/RosbagsPietro/' + bag_name_cbf_sim
bag_cbf_sim = rosbag.Bag(bag_path_cbf_sim)


x_list_list_cbf_sim = []
y_list_list_cbf_sim = []
v_x_list_list_cbf_sim = []
v_y_list_list_cbf_sim = []
time_sec_list_list_cbf_sim = []
time_nsec_list_list_cbf_sim = []

v_des_x_list_list_cbf_sim = []
v_des_y_list_list_cbf_sim = []
des_time_sec_list_list_cbf_sim = []
des_time_nsec_list_list_cbf_sim = []

abs_vel_list_list_cbf_sim = []
abs_vel_des_list_list_cbf_sim = []


h_fun_list_list_sim = []
h_p_list_list_sim = []
min_alpha_h_list_list_sim = []

time_h_sec_list_list_sim = []
time_h_nsec_list_list_sim = []


for ii in range(N_cf):
    # position trajectories
    x_list_i = []
    x_list_list_cbf_sim.append(x_list_i)
    y_list_i = []
    y_list_list_cbf_sim.append(y_list_i)
    # velocity trajectories
    v_x_list_i = []
    v_x_list_list_cbf_sim.append(v_x_list_i)
    v_y_list_i = []
    v_y_list_list_cbf_sim.append(v_y_list_i)
    # desired velocity trajectories
    v_des_x_list_i = []
    v_des_x_list_list_cbf_sim.append(v_des_x_list_i)
    v_des_y_list_i = []
    v_des_y_list_list_cbf_sim.append(v_des_y_list_i)
    # time
    time_sec_list_i = []
    time_sec_list_list_cbf_sim.append(time_sec_list_i)
    time_nsec_list_i = []
    time_nsec_list_list_cbf_sim.append(time_nsec_list_i)
    # time des vel
    des_time_sec_list_i = []
    des_time_sec_list_list_cbf_sim.append(des_time_sec_list_i)
    des_time_nsec_list_i = []
    des_time_nsec_list_list_cbf_sim.append(des_time_nsec_list_i)
    # h_fun history
    h_fun_list_i = []
    h_fun_list_list_sim.append(h_fun_list_i)
    # h_p history
    h_p_list_i = []
    h_p_list_list_sim.append(h_p_list_i)
    # alpha_h history
    alpha_h_list_i = []
    min_alpha_h_list_list_sim.append(alpha_h_list_i)
    # time h
    time_h_sec_list_i = []
    time_h_sec_list_list_sim.append(time_h_sec_list_i)
    time_h_nsec_list_i = []
    time_h_nsec_list_list_sim.append(time_h_nsec_list_i)



for ii in range(N_cf):
    for topic, msg, t in bag_cbf_sim.read_messages(topics=['/cf'+ str(ii+1) +'/state']):
        # position trajectories
        x_list_list_cbf_sim[ii].append(msg.position.x)
        y_list_list_cbf_sim[ii].append(msg.position.y)
        # velocity trajectories
        v_x_list_list_cbf_sim[ii].append(msg.velocity.x)
        v_y_list_list_cbf_sim[ii].append(msg.velocity.y)
        # time
        time_sec_list_list_cbf_sim[ii].append(t.secs)
        time_nsec_list_list_cbf_sim[ii].append(t.nsecs)

    # putting together seconds and nanoseconds
    nsec_to_sec = [x * 10**(-9) for x in time_nsec_list_list_cbf_sim[ii]]
    time_sec_list_list_cbf_sim[ii] = np.add(time_sec_list_list_cbf_sim[ii], nsec_to_sec)
    term_subtract = time_sec_list_list_cbf_sim[ii][0]
    time_sec_list_list_cbf_sim[ii] = np.subtract(time_sec_list_list_cbf_sim[ii], term_subtract)


    # print('time_sec_list_list[ii] is: ', time_sec_list_list_cbf_sim[ii])
    # print('length time_sec_list_list[ii] is: ', len(time_sec_list_list_cbf_sim[ii]))

for ii in range(N_cf):
    for topic, msg, t in bag_cbf_sim.read_messages(topics=['/cf'+ str(ii+1) +'/actual_state_target']):
    # for topic, msg, t in bag.read_messages(topics=['/cf'+ str(ii+1) +'/mpc_velocity']):
        # desired velocity trajectories
        v_des_x_list_list_cbf_sim[ii].append(msg.desired_velocity.x)
        v_des_y_list_list_cbf_sim[ii].append(msg.desired_velocity.y)
        # time
        des_time_sec_list_list_cbf_sim[ii].append(t.secs)
        des_time_nsec_list_list_cbf_sim[ii].append(t.nsecs)

    # putting together seconds and nanoseconds
    nsec_to_sec = [x * 10**(-9) for x in des_time_nsec_list_list_cbf_sim[ii]]
    des_time_sec_list_list_cbf_sim[ii] = np.add(des_time_sec_list_list_cbf_sim[ii], nsec_to_sec)
    des_time_sec_list_list_cbf_sim[ii] = np.subtract(des_time_sec_list_list_cbf_sim[ii], term_subtract)


    x = np.array(v_des_x_list_list_cbf_sim[ii])
    indices_of_zeros_cbf_sim = np.where(x == 0)[0]
    print('indices of zeros cbf is: ', indices_of_zeros_cbf_sim)
    time_start_cbf_sim = des_time_sec_list_list_cbf_sim[ii][len(indices_of_zeros_cbf_sim)] + 0.2
    print('time_start_cbf_sim is: ', time_start_cbf_sim)



for ii in range(N_cf):
    for topic, msg, t in bag_cbf_sim.read_messages(topics=['/cf'+ str(ii+1) +'/cbf_function']):
        # desired velocity trajectories
        h_fun_list_list_sim[ii].append(msg.desired_position.z)
        h_p_list_list_sim[ii].append(msg.desired_position.y)
        min_alpha_h_list_list_sim[ii].append(msg.desired_position.x)
        # time
        time_h_sec_list_list_sim[ii].append(t.secs)
        time_h_nsec_list_list_sim[ii].append(t.nsecs)

    # putting together seconds and nanoseconds
    nsec_to_sec = [x * 10**(-9) for x in time_h_nsec_list_list_sim[ii]]
    time_h_sec_list_list_sim[ii] = np.add(time_h_sec_list_list_sim[ii], nsec_to_sec)
    time_h_sec_list_list_sim[ii] = np.subtract(time_h_sec_list_list_sim[ii], term_subtract)


# +++++++++++++++++++++ Extracting data about cbf real +++++++++++++++++++++++++++++++++

bag_name_cbf_real = '1_drone_cbf_real_test4.bag'
bag_path_cbf_real = rospack.get_path('crazyCmd') + '/data/output/RosbagsPietro/' + bag_name_cbf_real
bag_cbf_real = rosbag.Bag(bag_path_cbf_real)


x_list_list_cbf_real = []
y_list_list_cbf_real = []
v_x_list_list_cbf_real = []
v_y_list_list_cbf_real = []
time_sec_list_list_cbf_real = []
time_nsec_list_list_cbf_real = []

v_des_x_list_list_cbf_real = []
v_des_y_list_list_cbf_real = []
des_time_sec_list_list_cbf_real = []
des_time_nsec_list_list_cbf_real = []

abs_vel_list_list_cbf_real = []
abs_vel_des_list_list_cbf_real = []


h_fun_list_list_real = []
h_p_list_list_real = []
min_alpha_h_list_list_real = []

time_h_sec_list_list_real = []
time_h_nsec_list_list_real = []


for ii in range(N_cf):
    # position trajectories
    x_list_i = []
    x_list_list_cbf_real.append(x_list_i)
    y_list_i = []
    y_list_list_cbf_real.append(y_list_i)
    # velocity trajectories
    v_x_list_i = []
    v_x_list_list_cbf_real.append(v_x_list_i)
    v_y_list_i = []
    v_y_list_list_cbf_real.append(v_y_list_i)
    # desired velocity trajectories
    v_des_x_list_i = []
    v_des_x_list_list_cbf_real.append(v_des_x_list_i)
    v_des_y_list_i = []
    v_des_y_list_list_cbf_real.append(v_des_y_list_i)
    # time
    time_sec_list_i = []
    time_sec_list_list_cbf_real.append(time_sec_list_i)
    time_nsec_list_i = []
    time_nsec_list_list_cbf_real.append(time_nsec_list_i)
    # time des vel
    des_time_sec_list_i = []
    des_time_sec_list_list_cbf_real.append(des_time_sec_list_i)
    des_time_nsec_list_i = []
    des_time_nsec_list_list_cbf_real.append(des_time_nsec_list_i)
    # h_fun history
    h_fun_list_i = []
    h_fun_list_list_real.append(h_fun_list_i)
    # h_p history
    h_p_list_i = []
    h_p_list_list_real.append(h_p_list_i)
    # alpha_h history
    alpha_h_list_i = []
    min_alpha_h_list_list_real.append(alpha_h_list_i)
    # time h
    time_h_sec_list_i = []
    time_h_sec_list_list_real.append(time_h_sec_list_i)
    time_h_nsec_list_i = []
    time_h_nsec_list_list_real.append(time_h_nsec_list_i)



for ii in range(N_cf):
    for topic, msg, t in bag_cbf_real.read_messages(topics=['/cf'+ str(ii+1) +'/state']):
        # position trajectories
        x_list_list_cbf_real[ii].append(msg.position.x)
        y_list_list_cbf_real[ii].append(msg.position.y)
        # velocity trajectories
        v_x_list_list_cbf_real[ii].append(msg.velocity.x)
        v_y_list_list_cbf_real[ii].append(msg.velocity.y)
        # time
        time_sec_list_list_cbf_real[ii].append(t.secs)
        time_nsec_list_list_cbf_real[ii].append(t.nsecs)

    # putting together seconds and nanoseconds
    nsec_to_sec = [x * 10**(-9) for x in time_nsec_list_list_cbf_real[ii]]
    time_sec_list_list_cbf_real[ii] = np.add(time_sec_list_list_cbf_real[ii], nsec_to_sec)
    term_subtract = time_sec_list_list_cbf_real[ii][0]
    time_sec_list_list_cbf_real[ii] = np.subtract(time_sec_list_list_cbf_real[ii], term_subtract)


    # print('time_sec_list_list[ii] is: ', time_sec_list_list_cbf_real[ii])
    # print('length time_sec_list_list[ii] is: ', len(time_sec_list_list_cbf_real[ii]))

for ii in range(N_cf):
    for topic, msg, t in bag_cbf_real.read_messages(topics=['/cf'+ str(ii+1) +'/actual_state_target']):
    # for topic, msg, t in bag.read_messages(topics=['/cf'+ str(ii+1) +'/mpc_velocity']):
        # desired velocity trajectories
        v_des_x_list_list_cbf_real[ii].append(msg.desired_velocity.x)
        v_des_y_list_list_cbf_real[ii].append(msg.desired_velocity.y)
        # time
        des_time_sec_list_list_cbf_real[ii].append(t.secs)
        des_time_nsec_list_list_cbf_real[ii].append(t.nsecs)

    # putting together seconds and nanoseconds
    nsec_to_sec = [x * 10**(-9) for x in des_time_nsec_list_list_cbf_real[ii]]
    des_time_sec_list_list_cbf_real[ii] = np.add(des_time_sec_list_list_cbf_real[ii], nsec_to_sec)
    des_time_sec_list_list_cbf_real[ii] = np.subtract(des_time_sec_list_list_cbf_real[ii], term_subtract)


    x = np.array(v_des_x_list_list_cbf_real[ii])
    indices_of_zeros_cbf_real = np.where(x == 0)[0]
    print('indices of zeros cbf is: ', indices_of_zeros_cbf_real)
    time_start_cbf_real = des_time_sec_list_list_cbf_real[ii][len(indices_of_zeros_cbf_real)]
    print('time_start_cbf_real is: ', time_start_cbf_real)



for ii in range(N_cf):
    for topic, msg, t in bag_cbf_real.read_messages(topics=['/cf'+ str(ii+1) +'/cbf_function']):
        # desired velocity trajectories
        h_fun_list_list_real[ii].append(msg.desired_position.z)
        h_p_list_list_real[ii].append(msg.desired_position.y)
        min_alpha_h_list_list_real[ii].append(msg.desired_position.x)
        # time
        time_h_sec_list_list_real[ii].append(t.secs)
        time_h_nsec_list_list_real[ii].append(t.nsecs)

    # putting together seconds and nanoseconds
    nsec_to_sec = [x * 10**(-9) for x in time_h_nsec_list_list_real[ii]]
    time_h_sec_list_list_real[ii] = np.add(time_h_sec_list_list_real[ii], nsec_to_sec)
    time_h_sec_list_list_real[ii] = np.subtract(time_h_sec_list_list_real[ii], term_subtract)





# ++++++++++++++++++++++ Shifting time vectors +++++++++++++++++++++++++++++++++

if time_start_cbf_sim > time_start_cbf_real:
    diff = time_start_cbf_sim - time_start_cbf_real
    # print('diff 1')
    # delete_indices = np.arange(0, diff)
    for ii in range(N_cf):
        time_subtract = diff
        time_sec_list_list_cbf_sim[ii] = np.subtract(time_sec_list_list_cbf_sim[ii], 
                                                 time_subtract)
        x_= np.array(time_sec_list_list_cbf_sim[ii])
        indices_neg = np.where(x < 0)[0]
        time_sec_list_list_cbf_sim[ii] = np.delete(time_sec_list_list_cbf_sim[ii], indices_neg)
        x_list_list_cbf_sim[ii] = np.delete(x_list_list_cbf_sim[ii], indices_neg)
        y_list_list_cbf_sim[ii] = np.delete(y_list_list_cbf_sim[ii], indices_neg)
        v_x_list_list_cbf_sim[ii] = np.delete(v_x_list_list_cbf_sim[ii], indices_neg)
        v_y_list_list_cbf_sim[ii] = np.delete(v_y_list_list_cbf_sim[ii], indices_neg)


        des_time_sec_list_list_cbf_sim[ii] = np.subtract(des_time_sec_list_list_cbf_sim[ii], 
                                                     time_subtract)
        x_= np.array(des_time_sec_list_list_cbf_sim[ii])
        indices_neg = np.where(x < 0)[0]
        des_time_sec_list_list_cbf_sim[ii] = np.delete(des_time_sec_list_list_cbf_sim[ii], indices_neg)
        v_des_x_list_list_cbf_sim[ii] = np.delete(v_des_x_list_list_cbf_sim[ii], indices_neg)
        v_des_x_list_list_cbf_sim[ii] = np.delete(v_des_x_list_list_cbf_sim[ii], indices_neg)


        time_h_sec_list_list_sim[ii] = np.subtract(time_h_sec_list_list_sim[ii], 
                                                   time_subtract)
        x_= np.array(time_h_sec_list_list_sim[ii])
        indices_neg = np.where(x < 0)[0]
        time_h_sec_list_list_sim[ii] = np.delete(time_h_sec_list_list_sim[ii], indices_neg)

        h_fun_list_list_sim[ii] = np.delete(h_fun_list_list_sim[ii], indices_neg)
        h_p_list_list_sim[ii] = np.delete(h_p_list_list_sim[ii], indices_neg)
        min_alpha_h_list_list_sim[ii] = np.delete(min_alpha_h_list_list_sim[ii], indices_neg)


elif time_start_cbf_sim < time_start_cbf_real:
    diff = time_start_cbf_real - time_start_cbf_sim
    # print('diff 2')
    # print('diff 2 is: ', diff)
    # delete_indices = np.arange(0, diff)
    for ii in range(N_cf):
        time_subtract = diff
        time_sec_list_list_cbf_real[ii] = np.subtract(time_sec_list_list_cbf_real[ii], 
                                                 time_subtract)
        print('time_sec_list_list_cbf_real is: ', time_sec_list_list_cbf_real[ii])
        x = np.array(time_sec_list_list_cbf_real[ii])
        print('x: ', x)
        indices_neg = np.where(x < 0)[0]
        print('indices_neg is: ', indices_neg)
        time_sec_list_list_cbf_real[ii] = np.delete(time_sec_list_list_cbf_real[ii], indices_neg)
        x_list_list_cbf_real[ii] = np.delete(x_list_list_cbf_real[ii], indices_neg)
        y_list_list_cbf_real[ii] = np.delete(y_list_list_cbf_real[ii], indices_neg)
        v_x_list_list_cbf_real[ii] = np.delete(v_x_list_list_cbf_real[ii], indices_neg)
        v_y_list_list_cbf_real[ii] = np.delete(v_y_list_list_cbf_real[ii], indices_neg)



        des_time_sec_list_list_cbf_real[ii] = np.subtract(des_time_sec_list_list_cbf_real[ii], 
                                                     time_subtract)
        x = np.array(des_time_sec_list_list_cbf_real[ii])
        indices_neg = np.where(x < 0)[0]
        des_time_sec_list_list_cbf_real[ii] = np.delete(des_time_sec_list_list_cbf_real[ii], indices_neg)
        v_des_x_list_list_cbf_real[ii] = np.delete(v_des_x_list_list_cbf_real[ii], indices_neg)
        v_des_y_list_list_cbf_real[ii] = np.delete(v_des_y_list_list_cbf_real[ii], indices_neg)


        time_h_sec_list_list_real[ii] = np.subtract(time_h_sec_list_list_real[ii], 
                                                    time_subtract)
        x_= np.array(time_h_sec_list_list_real[ii])
        indices_neg = np.where(x < 0)[0]
        time_h_sec_list_list_real[ii] = np.delete(time_h_sec_list_list_real[ii], indices_neg)

        h_fun_list_list_real[ii] = np.delete(h_fun_list_list_real[ii], indices_neg)
        h_p_list_list_real[ii] = np.delete(h_p_list_list_real[ii], indices_neg)
        min_alpha_h_list_list_real[ii] = np.delete(min_alpha_h_list_list_real[ii], indices_neg)


else:
    pass


# print('des_time_sec_list_list_cbf_real is: ', des_time_sec_list_list_cbf_real[ii])
# print('des_time_sec_list_list_cbf_sim is: ', des_time_sec_list_list_cbf_sim[ii])
# print('time_sec_list_list_cbf_real is: ', time_sec_list_list_cbf_real[ii])
# print('time_sec_list_list_cbf_sim is: ', time_sec_list_list_cbf_sim[ii])

print('len of v_des_x_list_list_cbf_real is: ', len(v_des_x_list_list_cbf_real[ii]))
print('len of v_des_x_list_list_cbf_real is: ', len(v_des_x_list_list_cbf_sim[ii]))



# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#                              P L O T T I N G

# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

# +++++++++++++++++++++++++ Target Coordinates +++++++++++++++++++++++++++++++++

x_target = 2.0
y_target = 0.0

# +++++++++++++++++++++++++ Plotting trajectories ++++++++++++++++++++++++++++++

fig1,ax1 = plt.subplots()
legend_traj = []

for ii in range(N_cf):
    ax1.plot(x_list_list_cbf_real[ii], y_list_list_cbf_real[ii])
    ax1.plot(x_list_list_cbf_sim[ii], y_list_list_cbf_sim[ii])
    legend_traj.append('drone_'+str(ii+1) + ' cbf real trajectory')
    legend_traj.append('drone_'+str(ii+1) + ' cbf sim trajectory')


ax1.set_xlabel('x [m]')

ax1.set_ylabel('y [m]')

ax1.set_aspect("equal")
plt.grid("minor")

# ++++++++++++++++++++++++++ Plotting Target +++++++++++++++++++++++++++++++++

ax1.plot(x_target, y_target, 'ko')

legend_traj.append('target')


ax1.legend(legend_traj)

# +++++++++++++++++++++++++ Plotting Obstacles +++++++++++++++++++++++++++++++


dummy_angle = np.linspace(0,2*np.pi,100)

# x_obs = [2.0, 1.0, 2.0]
# y_obs = [2.5, 2.5, 4.0]
# r_obs = [0.3, 0.2, 0.4]


x_obs = [1.0]
y_obs = [0.1]
r_obs = [0.3]
r_obs_safe = [0.4]

for ii in range(N_obs):
    ax1.plot(x_obs[ii] + r_obs[ii]*np.cos(dummy_angle),
             y_obs[ii] + r_obs[ii]*np.sin(dummy_angle), 'k')



# +++++++++++++++++++++ Plotting velocity vs. time +++++++++++++++++++++++++++


fig2,(ax10, ax2) = plt.subplots(1,2, sharey='row')
legend_vel = []

for ii in range(N_cf):
    # Actual velocities
    vel_x_list_i = np.array(v_x_list_list_cbf_sim[ii])
    vel_y_list_i = np.array(v_y_list_list_cbf_sim[ii])
    # Desired velocities fed to velocity controller
    vel_des_x_list_i = np.array(v_des_x_list_list_cbf_sim[ii])
    vel_des_y_list_i = np.array(v_des_y_list_list_cbf_sim[ii])
    # Absolute actual velocity
    abs_vel_list_i = np.sqrt(np.add(np.square(vel_x_list_i), np.square(vel_y_list_i)))
    # Absolute desired velocity
    abs_vel_des_list_i = np.sqrt(np.add(np.square(vel_des_x_list_i), np.square(vel_des_y_list_i)))

    abs_vel_list_list_cbf_sim.append(abs_vel_list_i)
    abs_vel_des_list_list_cbf_sim.append(abs_vel_des_list_i)


for ii in range(N_cf):
    vel_x_list_i = np.array(v_x_list_list_cbf_real[ii])
    vel_y_list_i = np.array(v_y_list_list_cbf_real[ii])
    vel_des_x_list_i = np.array(v_des_x_list_list_cbf_real[ii])
    vel_des_y_list_i = np.array(v_des_y_list_list_cbf_real[ii])
    
    abs_vel_list_i = np.sqrt(np.add(np.square(vel_x_list_i), np.square(vel_y_list_i)))
    abs_vel_des_list_i = np.sqrt(np.add(np.square(vel_des_x_list_i), np.square(vel_des_y_list_i)))

    # print('length of abs_vel_list_i is: ', len(abs_vel_list_i))

    abs_vel_list_list_cbf_real.append(abs_vel_list_i)
    abs_vel_des_list_list_cbf_real.append(abs_vel_des_list_i)

for ii in range(N_cf):
    ax2.plot(time_sec_list_list_cbf_real[ii], abs_vel_list_list_cbf_real[ii])
    ax2.plot(time_sec_list_list_cbf_sim[ii], abs_vel_list_list_cbf_sim[ii])
    legend_vel.append('v_' + str(ii+1) + ' cbf real')
    legend_vel.append('v_' + str(ii+1) + ' cbf sim')

ax2.legend(legend_vel)

ax2.set_xlabel('t [s]')
ax2.set_ylabel('v [m/s]')
ax2.set_title('Actual velocity')

ax2.grid("minor")

# fig10,ax10 = plt.subplots()
legend_vel_cbf_real = []

for ii in range(N_cf):
    ax10.plot(des_time_sec_list_list_cbf_real[ii], abs_vel_des_list_list_cbf_real[ii])
    ax10.plot(des_time_sec_list_list_cbf_sim[ii], abs_vel_des_list_list_cbf_sim[ii])
    legend_vel_cbf_real.append('v_' + str(ii+1) + ' cbf real')
    legend_vel_cbf_real.append('v_' + str(ii+1) + ' cbf sim')

ax10.legend(legend_vel_cbf_real)

ax10.set_xlabel('t [s]')
ax10.set_ylabel('v [m/s]')
ax10.set_title('Desired velocity')

ax10.grid("minor")



# ++++++++++ Plotting Velocity Trajectories - Actual vs. Desired ++++++++++++++

fig5,(ax14,ax5) = plt.subplots(1,2)
legend_vel_traj = []

for ii in range(N_cf):
    ax5.plot(v_x_list_list_cbf_real[ii], v_y_list_list_cbf_real[ii])
    ax5.plot(v_x_list_list_cbf_sim[ii], v_y_list_list_cbf_sim[ii])

    legend_vel_traj.append('drone_'+str(ii+1)+' actual velocity trajectory real')
    legend_vel_traj.append('drone_'+str(ii+1)+' actual velocity trajectory sim')


ax5.legend(legend_vel_traj)
ax5.set_xlabel('v_x [m]')

ax5.set_ylabel('v_y [m]')

ax5.set_aspect("equal")

ax5.set_title("Actual velocity")

ax5.grid("minor")

# fig14,ax14 = plt.subplots()
legend_vel_traj = []

for ii in range(N_cf):
    ax14.plot(v_des_x_list_list_cbf_real[ii], v_des_y_list_list_cbf_real[ii])
    ax14.plot(v_des_x_list_list_cbf_sim[ii], v_des_y_list_list_cbf_sim[ii])

    legend_vel_traj.append('drone_' + str(ii+1) + ' desired velocity trajectory real')
    legend_vel_traj.append('drone_' + str(ii+1) + ' desired velocity trajectory sim')

ax14.legend(legend_vel_traj)
ax14.set_xlabel('v_x [m]')

ax14.set_ylabel('v_y [m]')

ax14.set_aspect("equal")

ax14.set_title("Desired velocity")

ax14.grid("minor")






# ++++++++++++++++++++ Plotting h cbf function ++++++++++++++++++++++++++++++

fig7,ax7 = plt.subplots()

legend_h = []

for ii in range(N_cf):
    ax7.plot(time_h_sec_list_list_sim[ii], h_fun_list_list_sim[ii])
    ax7.plot(time_h_sec_list_list_real[ii], h_fun_list_list_real[ii])
    legend_h.append('h sim')
    legend_h.append('h real')

ax7.legend(legend_h)
ax7.set_xlabel('time [s]')

ax7.set_ylabel('h')

ax7.grid("minor")
# ax7.set_title('h function')







# ++++++++++++++++++++++++ Plotting constraint ++++++++++++++++++++++++++++++

fig9,(ax99,ax9) = plt.subplots(1,2, sharey='row')

legend_h_p = []
legend_h_p99 = []

for ii in range(N_cf):
    ax9.plot(time_h_sec_list_list_sim[ii], h_p_list_list_sim[ii])
    ax9.plot(time_h_sec_list_list_sim[ii], min_alpha_h_list_list_sim[ii], '--')
    ax99.plot(time_h_sec_list_list_real[ii], h_p_list_list_real[ii])
    ax99.plot(time_h_sec_list_list_real[ii], min_alpha_h_list_list_real[ii], '--')
    legend_h_p.append('h_p sim')
    legend_h_p.append('-alpha*h sim')
    legend_h_p99.append('h_p real')
    legend_h_p99.append('-alpha*h real')



ax9.set_xlabel('time [s]')

ax9.set_ylabel('h_p vs -alpha*h')
ax99.set_xlabel('time [s]')

ax99.set_ylabel('h_p vs -alpha*h')
# ax9.set_title('constraint')

ax9.legend(legend_h_p)
ax99.legend(legend_h_p99)

ax9.grid("minor")
ax99.grid("minor")

ax9.set_title("Simulation")
ax99.set_title("Reality")


# +++++++++++++ Plotting distance of Center of Mass from Target ++++++++++++++



distance_cm_list_cbf_real = []
N_time_steps_cm_cbf_real = len(x_list_list_cbf_real[0])
# print('N_time_steps_cm_cbf_real is: ', N_time_steps_cm_cbf_real)
legend_distance_cm = []
x_cm_list_cbf_real = []
y_cm_list_cbf_real = []

for kk in range(N_time_steps_cm_cbf_real):
    x_cm_sum = 0
    y_cm_sum = 0
    for ii in range(N_cf):
        x_cm_sum += x_list_list_cbf_real[ii][kk]
        y_cm_sum += y_list_list_cbf_real[ii][kk]
    x_cm = x_cm_sum/N_cf
    y_cm = y_cm_sum/N_cf
    x_cm_list_cbf_real.append(x_cm)
    y_cm_list_cbf_real.append(y_cm)



for kk in range(N_time_steps_cm_cbf_real):
    dist = sqrt((x_cm_list_cbf_real[kk] - x_target)**2 + (y_cm_list_cbf_real[kk] - y_target)**2)
    distance_cm_list_cbf_real.append(dist)

legend_distance_cm.append('absolute distance cbf real')
fig15,ax15 = plt.subplots()


ax15.set_xlabel('t [s]')
ax15.set_ylabel('d [m]')


ax15.plot(time_sec_list_list_cbf_real[0], distance_cm_list_cbf_real)



distance_cm_list_cbf_sim = []
N_time_steps_cm_cbf_sim = len(x_list_list_cbf_sim[0])
# print('N_time_steps_cm_cbf_real is: ', N_time_steps_cm_cbf_sim)
legend_distance_cbf_sim = []
x_cm_list_cbf_sim = []
y_cm_list_cbf_sim = []

for kk in range(N_time_steps_cm_cbf_sim):
    x_cm_sum = 0
    y_cm_sum = 0
    for ii in range(N_cf):
        x_cm_sum += x_list_list_cbf_sim[ii][kk]
        y_cm_sum += y_list_list_cbf_sim[ii][kk]
    x_cm = x_cm_sum/N_cf
    y_cm = y_cm_sum/N_cf
    x_cm_list_cbf_sim.append(x_cm)
    y_cm_list_cbf_sim.append(y_cm)



for kk in range(N_time_steps_cm_cbf_sim):
    dist = sqrt((x_cm_list_cbf_sim[kk] - x_target)**2 + (y_cm_list_cbf_sim[kk] - y_target)**2)
    distance_cm_list_cbf_sim.append(dist)

legend_distance_cm.append('absolute distance cbf sim')


ax15.set_xlabel('t [s]')
ax15.set_ylabel('d [m]')


ax15.plot(time_sec_list_list_cbf_sim[0], distance_cm_list_cbf_sim)

ax15.legend(legend_distance_cm)




plt.grid("minor")




# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
print('h_fun_list_list_real is: ', h_fun_list_list_real[ii])



plt.show(block=True)

bag_cbf_sim.close()
bag_cbf_real.close()