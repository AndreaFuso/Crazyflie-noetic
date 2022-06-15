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
N_mpc = 5
mpc_traj_flag = True

rospack = rospkg.RosPack()

# +++++++++++++++++++++ Extracting data about mpc1 +++++++++++++++++++++++++++++++++


bag_name_mpc_real = '1_drone_mpc_real_test6.bag'
bag_path_mpc_real = rospack.get_path('crazyCmd') + '/data/output/RosbagsPietro/' + bag_name_mpc_real
bag_mpc_real = rosbag.Bag(bag_path_mpc_real)


x_list_list_mpc_real = []
y_list_list_mpc_real = []
v_x_list_list_mpc_real = []
v_y_list_list_mpc_real = []
time_sec_list_list_mpc_real = []
time_nsec_list_list_mpc_real = []

v_des_x_list_list_mpc_real = []
v_des_y_list_list_mpc_real = []
des_time_sec_list_list_mpc_real = []
des_time_nsec_list_list_mpc_real = []

abs_vel_list_list_mpc_real = []
abs_vel_des_list_list_mpc_real = []

time_mpc_sec_list_list_real = []
time_mpc_nsec_list_list_real = []
mpc_x_list_list_real = []
mpc_y_list_list_real = []


for ii in range(N_cf):
    # position trajectories
    x_list_i = []
    x_list_list_mpc_real.append(x_list_i)
    y_list_i = []
    y_list_list_mpc_real.append(y_list_i)
    # velocity trajectories
    v_x_list_i = []
    v_x_list_list_mpc_real.append(v_x_list_i)
    v_y_list_i = []
    v_y_list_list_mpc_real.append(v_y_list_i)
    # desired velocity trajectories
    v_des_x_list_i = []
    v_des_x_list_list_mpc_real.append(v_des_x_list_i)
    v_des_y_list_i = []
    v_des_y_list_list_mpc_real.append(v_des_y_list_i)
    # time
    time_sec_list_i = []
    time_sec_list_list_mpc_real.append(time_sec_list_i)
    time_nsec_list_i = []
    time_nsec_list_list_mpc_real.append(time_nsec_list_i)
    # time des vel
    des_time_sec_list_i = []
    des_time_sec_list_list_mpc_real.append(des_time_sec_list_i)
    des_time_nsec_list_i = []
    des_time_nsec_list_list_mpc_real.append(des_time_nsec_list_i)
    # time mpc
    time_mpc_sec_list_i = []
    time_mpc_sec_list_list_real.append(time_mpc_sec_list_i)
    time_mpc_nsec_list_i = []
    time_mpc_nsec_list_list_real.append(time_mpc_nsec_list_i)
    # mpc position trajectories
    mpc_x_list_i = []
    mpc_x_list_list_real.append(mpc_x_list_i)
    mpc_y_list_i = []
    mpc_y_list_list_real.append(mpc_y_list_i)


for ii in range(N_cf):
    for topic, msg, t in bag_mpc_real.read_messages(topics=['/cf'+ str(ii+1) +'/state']):
        # position trajectories
        x_list_list_mpc_real[ii].append(msg.position.x)
        y_list_list_mpc_real[ii].append(msg.position.y)
        # velocity trajectories
        v_x_list_list_mpc_real[ii].append(msg.velocity.x)
        v_y_list_list_mpc_real[ii].append(msg.velocity.y)
        # time
        time_sec_list_list_mpc_real[ii].append(t.secs)
        time_nsec_list_list_mpc_real[ii].append(t.nsecs)

    # putting together seconds and nanoseconds
    nsec_to_sec_real = [x * 10**(-9) for x in time_nsec_list_list_mpc_real[ii]]
    time_sec_list_list_mpc_real[ii] = np.add(time_sec_list_list_mpc_real[ii], nsec_to_sec_real)
    term_subtract_real = time_sec_list_list_mpc_real[ii][0]
    time_sec_list_list_mpc_real[ii] = np.subtract(time_sec_list_list_mpc_real[ii], term_subtract_real)

    # print('time_sec_list_list[ii] is: ', time_sec_list_list_mpc[ii])

for ii in range(N_cf):
    for topic, msg, t in bag_mpc_real.read_messages(topics=['/cf'+ str(ii+1) +'/actual_state_target']):
        # desired velocity trajectories
        v_des_x_list_list_mpc_real[ii].append(msg.desired_velocity.x)
        v_des_y_list_list_mpc_real[ii].append(msg.desired_velocity.y)
        # time
        des_time_sec_list_list_mpc_real[ii].append(t.secs)
        des_time_nsec_list_list_mpc_real[ii].append(t.nsecs)
    
    # putting together seconds and nanoseconds
    nsec_to_sec_real = [x * 10**(-9) for x in des_time_nsec_list_list_mpc_real[ii]]
    des_time_sec_list_list_mpc_real[ii] = np.add(des_time_sec_list_list_mpc_real[ii], nsec_to_sec_real)
    des_time_sec_list_list_mpc_real[ii] = np.subtract(des_time_sec_list_list_mpc_real[ii], term_subtract_real)

    x = np.array(v_des_x_list_list_mpc_real[ii])
    indices_of_zeros_mpc_real = np.where(x == 0)[0]
    # print('indices of zeros mpc is: ', indices_of_zeros_mpc)
    time_start_mpc_real = des_time_sec_list_list_mpc_real[ii][len(indices_of_zeros_mpc_real)]
    print('time_start_mpc is: ', time_start_mpc_real)


for ii in range(N_cf):
    for topic, msg, t in bag_mpc_real.read_messages(topics=['/cf'+ str(ii+1) +'/mpc_traj']):
        # mpc optimal position trajectories
        mpc_x_list_j_real = []
        mpc_y_list_j_real = []
        for jj in range(N_mpc + 1):
            mpc_x_list_j_real.append(msg.x_vec[jj].desired_position.x)
            mpc_y_list_j_real.append(msg.x_vec[jj].desired_position.y)

        mpc_x_list_list_real[ii].append(mpc_x_list_j_real)
        mpc_y_list_list_real[ii].append(mpc_y_list_j_real)
        # time
        time_mpc_sec_list_list_real[ii].append(t.secs)
        time_mpc_nsec_list_list_real[ii].append(t.nsecs)



# +++++++++++++++++++++ Extracting data about mpc2 +++++++++++++++++++++++++++++++++


bag_name_mpc_sim = '1_drone_mpc_sim_test2.bag'
bag_path_mpc_sim = rospack.get_path('crazyCmd') + '/data/output/RosbagsPietro/' + bag_name_mpc_sim
bag_mpc_sim = rosbag.Bag(bag_path_mpc_sim)


x_list_list_mpc_sim = []
y_list_list_mpc_sim = []
v_x_list_list_mpc_sim = []
v_y_list_list_mpc_sim = []
time_sec_list_list_mpc_sim = []
time_nsec_list_list_mpc_sim = []

v_des_x_list_list_mpc_sim = []
v_des_y_list_list_mpc_sim = []
des_time_sec_list_list_mpc_sim = []
des_time_nsec_list_list_mpc_sim = []

abs_vel_list_list_mpc_sim = []
abs_vel_des_list_list_mpc_sim = []

time_mpc_sec_list_list_sim = []
time_mpc_nsec_list_list_sim = []
mpc_x_list_list_sim = []
mpc_y_list_list_sim = []


for ii in range(N_cf):
    # position trajectories
    x_list_i = []
    x_list_list_mpc_sim.append(x_list_i)
    y_list_i = []
    y_list_list_mpc_sim.append(y_list_i)
    # velocity trajectories
    v_x_list_i = []
    v_x_list_list_mpc_sim.append(v_x_list_i)
    v_y_list_i = []
    v_y_list_list_mpc_sim.append(v_y_list_i)
    # desired velocity trajectories
    v_des_x_list_i = []
    v_des_x_list_list_mpc_sim.append(v_des_x_list_i)
    v_des_y_list_i = []
    v_des_y_list_list_mpc_sim.append(v_des_y_list_i)
    # time
    time_sec_list_i = []
    time_sec_list_list_mpc_sim.append(time_sec_list_i)
    time_nsec_list_i = []
    time_nsec_list_list_mpc_sim.append(time_nsec_list_i)
    # time des vel
    des_time_sec_list_i = []
    des_time_sec_list_list_mpc_sim.append(des_time_sec_list_i)
    des_time_nsec_list_i = []
    des_time_nsec_list_list_mpc_sim.append(des_time_nsec_list_i)
    # time mpc
    time_mpc_sec_list_i = []
    time_mpc_sec_list_list_sim.append(time_mpc_sec_list_i)
    time_mpc_nsec_list_i = []
    time_mpc_nsec_list_list_sim.append(time_mpc_nsec_list_i)
    # mpc position trajectories
    mpc_x_list_i = []
    mpc_x_list_list_sim.append(mpc_x_list_i)
    mpc_y_list_i = []
    mpc_y_list_list_sim.append(mpc_y_list_i)


for ii in range(N_cf):
    for topic, msg, t in bag_mpc_sim.read_messages(topics=['/cf'+ str(ii+1) +'/state']):
        # position trajectories
        x_list_list_mpc_sim[ii].append(msg.position.x)
        y_list_list_mpc_sim[ii].append(msg.position.y)
        # velocity trajectories
        v_x_list_list_mpc_sim[ii].append(msg.velocity.x)
        v_y_list_list_mpc_sim[ii].append(msg.velocity.y)
        # time
        time_sec_list_list_mpc_sim[ii].append(t.secs)
        time_nsec_list_list_mpc_sim[ii].append(t.nsecs)

    # putting together seconds and nanoseconds
    nsec_to_sec_sim = [x * 10**(-9) for x in time_nsec_list_list_mpc_sim[ii]]
    time_sec_list_list_mpc_sim[ii] = np.add(time_sec_list_list_mpc_sim[ii], nsec_to_sec_sim)
    term_subtract_sim = time_sec_list_list_mpc_sim[ii][0]
    time_sec_list_list_mpc_sim[ii] = np.subtract(time_sec_list_list_mpc_sim[ii], term_subtract_sim)

    # print('time_sec_list_list[ii] is: ', time_sec_list_list_mpc[ii])

for ii in range(N_cf):
    for topic, msg, t in bag_mpc_sim.read_messages(topics=['/cf'+ str(ii+1) +'/actual_state_target']):
        # desired velocity trajectories
        v_des_x_list_list_mpc_sim[ii].append(msg.desired_velocity.x)
        v_des_y_list_list_mpc_sim[ii].append(msg.desired_velocity.y)
        # time
        des_time_sec_list_list_mpc_sim[ii].append(t.secs)
        des_time_nsec_list_list_mpc_sim[ii].append(t.nsecs)
    
    # putting together seconds and nanoseconds
    nsec_to_sec_sim = [x * 10**(-9) for x in des_time_nsec_list_list_mpc_sim[ii]]
    des_time_sec_list_list_mpc_sim[ii] = np.add(des_time_sec_list_list_mpc_sim[ii], nsec_to_sec_sim)
    des_time_sec_list_list_mpc_sim[ii] = np.subtract(des_time_sec_list_list_mpc_sim[ii], term_subtract_sim)

    x = np.array(v_des_x_list_list_mpc_sim[ii])
    indices_of_zeros_mpc_sim = np.where(x == 0)[0]
    # print('indices of zeros mpc is: ', indices_of_zeros_mpc)
    time_start_mpc_sim = des_time_sec_list_list_mpc_sim[ii][len(indices_of_zeros_mpc_sim)] + 0.2
    print('time_start_mpc is: ', time_start_mpc_sim)


for ii in range(N_cf):
    for topic, msg, t in bag_mpc_sim.read_messages(topics=['/cf'+ str(ii+1) +'/mpc_traj']):
        # mpc optimal position trajectories
        mpc_x_list_j_sim = []
        mpc_y_list_j_sim = []
        for jj in range(N_mpc + 1):
            mpc_x_list_j_sim.append(msg.x_vec[jj].desired_position.x)
            mpc_y_list_j_sim.append(msg.x_vec[jj].desired_position.y)

        mpc_x_list_list_sim[ii].append(mpc_x_list_j_sim)
        mpc_y_list_list_sim[ii].append(mpc_y_list_j_sim)
        # time
        time_mpc_sec_list_list_sim[ii].append(t.secs)
        time_mpc_nsec_list_list_sim[ii].append(t.nsecs)


# ++++++++++++++++++++++ Shifting time vectors +++++++++++++++++++++++++++++++++



if time_start_mpc_sim > time_start_mpc_real:
    diff = time_start_mpc_sim - time_start_mpc_real

    for ii in range(N_cf):
        time_subtract = diff
        time_sec_list_list_mpc_sim[ii] = np.subtract(time_sec_list_list_mpc_sim[ii], 
                                                     time_subtract)
        print('time_sec_list_list_mpc is: ', time_sec_list_list_mpc_sim[ii])
        x = np.array(time_sec_list_list_mpc_sim[ii])
        indices_neg = np.where(x < 0)[0]
        print('indices_neg is: ', indices_neg)
        time_sec_list_list_mpc_sim[ii] = np.delete(time_sec_list_list_mpc_sim[ii], indices_neg)
        x_list_list_mpc_sim[ii] = np.delete(x_list_list_mpc_sim[ii], indices_neg)
        y_list_list_mpc_sim[ii] = np.delete(y_list_list_mpc_sim[ii], indices_neg)
        v_x_list_list_mpc_sim[ii] = np.delete(v_x_list_list_mpc_sim[ii], indices_neg)
        v_y_list_list_mpc_sim[ii] = np.delete(v_y_list_list_mpc_sim[ii], indices_neg)

        des_time_sec_list_list_mpc_sim[ii] = np.subtract(des_time_sec_list_list_mpc_sim[ii], 
                                                         time_subtract)
        x = np.array(des_time_sec_list_list_mpc_sim[ii])
        indices_neg = np.where(x < 0)[0]
        des_time_sec_list_list_mpc_sim[ii] = np.delete(des_time_sec_list_list_mpc_sim[ii], indices_neg)
        v_des_x_list_list_mpc_sim[ii] = np.delete(v_des_x_list_list_mpc_sim[ii], indices_neg)
        v_des_y_list_list_mpc_sim[ii] = np.delete(v_des_y_list_list_mpc_sim[ii], indices_neg)

        print('des_time_sec_list_list_mpc_sim is: ', des_time_sec_list_list_mpc_sim[ii])

        print('des_time_sec_list_list_mpc_real is: ', des_time_sec_list_list_mpc_real[ii])

elif time_start_mpc_sim < time_start_mpc_real:
    diff = time_start_mpc_real - time_start_mpc_sim
    # print('diff 2')
    # print('diff 2 is: ', diff)
    # delete_indices = np.arange(0, diff)
    for ii in range(N_cf):
        time_subtract = diff
        time_sec_list_list_mpc_real[ii] = np.subtract(time_sec_list_list_mpc_real[ii], 
                                                      time_subtract)
        print('time_sec_list_list_mpc is: ', time_sec_list_list_mpc_real[ii])
        x = np.array(time_sec_list_list_mpc_real[ii])
        print('x: ', x)
        indices_neg = np.where(x < 0)[0]
        print('indices_neg is: ', indices_neg)
        time_sec_list_list_mpc_real[ii] = np.delete(time_sec_list_list_mpc_real[ii], indices_neg)
        x_list_list_mpc_real[ii] = np.delete(x_list_list_mpc_real[ii], indices_neg)
        y_list_list_mpc_real[ii] = np.delete(y_list_list_mpc_real[ii], indices_neg)
        v_x_list_list_mpc_real[ii] = np.delete(v_x_list_list_mpc_real[ii], indices_neg)
        v_y_list_list_mpc_real[ii] = np.delete(v_y_list_list_mpc_real[ii], indices_neg)



        des_time_sec_list_list_mpc_real[ii] = np.subtract(des_time_sec_list_list_mpc_real[ii], 
                                                     time_subtract)
        x = np.array(des_time_sec_list_list_mpc_real[ii])
        indices_neg = np.where(x < 0)[0]
        des_time_sec_list_list_mpc_real[ii] = np.delete(des_time_sec_list_list_mpc_real[ii], indices_neg)
        v_des_x_list_list_mpc_real[ii] = np.delete(v_des_x_list_list_mpc_real[ii], indices_neg)
        v_des_y_list_list_mpc_real[ii] = np.delete(v_des_y_list_list_mpc_real[ii], indices_neg)




        print('des_time_sec_list_list_mpc_sim is: ', des_time_sec_list_list_mpc_sim[ii])

        print('des_time_sec_list_list_mpc_real is: ', des_time_sec_list_list_mpc_real[ii])
else:
    pass


# print('des_time_sec_list_list_mpc is: ', des_time_sec_list_list_mpc[ii])
# print('des_time_sec_list_list_cbf is: ', des_time_sec_list_list_cbf[ii])
# print('time_sec_list_list_mpc is: ', time_sec_list_list_mpc[ii])
# print('time_sec_list_list_cbf is: ', time_sec_list_list_cbf[ii])




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
    ax1.plot(x_list_list_mpc_real[ii], y_list_list_mpc_real[ii])
    ax1.plot(x_list_list_mpc_sim[ii], y_list_list_mpc_sim[ii])
    legend_traj.append('drone_'+str(ii+1) + ' mpc real trajectory')
    legend_traj.append('drone_'+str(ii+1) + ' mpc sim trajectory')


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
    vel_x_list_i = np.array(v_x_list_list_mpc_sim[ii])
    vel_y_list_i = np.array(v_y_list_list_mpc_sim[ii])
    # Desired velocities fed to velocity controller
    vel_des_x_list_i = np.array(v_des_x_list_list_mpc_sim[ii])
    vel_des_y_list_i = np.array(v_des_y_list_list_mpc_sim[ii])
    # Absolute actual velocity
    abs_vel_list_i = np.sqrt(np.add(np.square(vel_x_list_i), np.square(vel_y_list_i)))
    # Absolute desired velocity
    abs_vel_des_list_i = np.sqrt(np.add(np.square(vel_des_x_list_i), np.square(vel_des_y_list_i)))

    abs_vel_list_list_mpc_sim.append(abs_vel_list_i)
    abs_vel_des_list_list_mpc_sim.append(abs_vel_des_list_i)

for ii in range(N_cf):
    vel_x_list_i = np.array(v_x_list_list_mpc_real[ii])
    vel_y_list_i = np.array(v_y_list_list_mpc_real[ii])
    vel_des_x_list_i = np.array(v_des_x_list_list_mpc_real[ii])
    vel_des_y_list_i = np.array(v_des_y_list_list_mpc_real[ii])
    
    abs_vel_list_i = np.sqrt(np.add(np.square(vel_x_list_i), np.square(vel_y_list_i)))
    abs_vel_des_list_i = np.sqrt(np.add(np.square(vel_des_x_list_i), np.square(vel_des_y_list_i)))

    # print('length of abs_vel_list_i is: ', len(abs_vel_list_i))

    abs_vel_list_list_mpc_real.append(abs_vel_list_i)
    abs_vel_des_list_list_mpc_real.append(abs_vel_des_list_i)

for ii in range(N_cf):
    ax2.plot(time_sec_list_list_mpc_real[ii], abs_vel_list_list_mpc_real[ii])
    ax2.plot(time_sec_list_list_mpc_sim[ii], abs_vel_list_list_mpc_sim[ii])
    legend_vel.append('v_' + str(ii+1) + ' mpc real')
    legend_vel.append('v_' + str(ii+1) + ' mpc sim')

ax2.legend(legend_vel)

ax2.set_xlabel('t [s]')
ax2.set_ylabel('v [m/s]')
ax2.set_title('Actual velocity')

ax2.grid("minor")

# fig10,ax10 = plt.subplots()
legend_vel_mpc = []

for ii in range(N_cf):
    ax10.plot(des_time_sec_list_list_mpc_real[ii], abs_vel_des_list_list_mpc_real[ii])
    ax10.plot(des_time_sec_list_list_mpc_sim[ii], abs_vel_des_list_list_mpc_sim[ii])
    legend_vel_mpc.append('v_des_' + str(ii+1) + ' mpc real')
    legend_vel_mpc.append('v_des_' + str(ii+1) + ' mpc sim')

ax10.legend(legend_vel_mpc)

ax10.set_xlabel('t [s]')
ax10.set_ylabel('v [m/s]')
ax10.set_title('Desired velocity')

ax10.grid("minor")



# ++++++++++ Plotting Velocity Trajectories - Actual and Desired ++++++++++++++

fig5,(ax14,ax5) = plt.subplots(1,2)
legend_vel_traj = []

for ii in range(N_cf):
    ax5.plot(v_x_list_list_mpc_real[ii], v_y_list_list_mpc_real[ii])
    ax5.plot(v_x_list_list_mpc_sim[ii], v_y_list_list_mpc_sim[ii])

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
    ax14.plot(v_des_x_list_list_mpc_real[ii], v_des_y_list_list_mpc_real[ii])
    ax14.plot(v_des_x_list_list_mpc_sim[ii], v_des_y_list_list_mpc_sim[ii])

    legend_vel_traj.append('drone_' + str(ii+1) + ' desired velocity trajectory real')
    legend_vel_traj.append('drone_' + str(ii+1) + ' desired velocity trajectory sim')

ax14.legend(legend_vel_traj)
ax14.set_xlabel('v_x [m]')

ax14.set_ylabel('v_y [m]')

ax14.set_aspect("equal")

ax14.set_title("Desired velocity")

ax14.grid("minor")



# +++++++++++++ Plotting distance of Center of Mass from Target ++++++++++++++


distance_cm_list_mpc_real = []
N_time_steps_cm_mpc_real = len(x_list_list_mpc_real[0])
# print('N_time_steps_cm_mpc is: ', N_time_steps_cm_mpc)
legend_distance_cm_real = []
x_cm_list_mpc_real = []
y_cm_list_mpc_real = []

for kk in range(N_time_steps_cm_mpc_real):
    x_cm_sum = 0
    y_cm_sum = 0
    for ii in range(N_cf):
        x_cm_sum += x_list_list_mpc_real[ii][kk]
        y_cm_sum += y_list_list_mpc_real[ii][kk]
    x_cm = x_cm_sum/N_cf
    y_cm = y_cm_sum/N_cf
    x_cm_list_mpc_real.append(x_cm)
    y_cm_list_mpc_real.append(y_cm)



for kk in range(N_time_steps_cm_mpc_real):
    dist = sqrt((x_cm_list_mpc_real[kk] - x_target)**2 + (y_cm_list_mpc_real[kk] - y_target)**2)
    distance_cm_list_mpc_real.append(dist)

legend_distance_cm_real.append('absolute distance mpc real')
fig15,ax15 = plt.subplots()


ax15.set_xlabel('t [s]')
ax15.set_ylabel('d [m]')


ax15.plot(time_sec_list_list_mpc_real[0], distance_cm_list_mpc_real)


distance_cm_list_mpc_sim = []
N_time_steps_cm_mps_sim = len(x_list_list_mpc_sim[0])
# print('N_time_steps_cm_mpc is: ', N_time_steps_cm_cbf)
legend_distance_mpc_sim = []
x_cm_list_mpc_sim = []
y_cm_list_mps_sim = []

for kk in range(N_time_steps_cm_mps_sim):
    x_cm_sum = 0
    y_cm_sum = 0
    for ii in range(N_cf):
        x_cm_sum += x_list_list_mpc_sim[ii][kk]
        y_cm_sum += y_list_list_mpc_sim[ii][kk]
    x_cm = x_cm_sum/N_cf
    y_cm = y_cm_sum/N_cf
    x_cm_list_mpc_sim.append(x_cm)
    y_cm_list_mps_sim.append(y_cm)



for kk in range(N_time_steps_cm_mps_sim):
    dist = sqrt((x_cm_list_mpc_sim[kk] - x_target)**2 + (y_cm_list_mps_sim[kk] - y_target)**2)
    distance_cm_list_mpc_sim.append(dist)

legend_distance_cm_real.append('absolute distance mpc sim')

ax15.plot(time_sec_list_list_mpc_sim[0], distance_cm_list_mpc_sim)

ax15.legend(legend_distance_cm_real)




plt.grid("minor")


# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


plt.show(block=True)

bag_mpc_real.close()
bag_mpc_sim.close()