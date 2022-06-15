#! /usr/bin/env python3

from cmath import sqrt
from tkinter import N
import rosbag
import rospkg
import matplotlib.pyplot as plt
import numpy as np

from crazy_common_py.common_functions import rad2deg


N_cf = 1
N_obs = 1
N_mpc = 5
mpc_traj_flag = True

rospack = rospkg.RosPack()


bag_name_mpc = '1_drone_mpc_real_test6.bag'
bag_path_mpc = rospack.get_path('crazyCmd') + '/data/output/RosbagsPietro/' + bag_name_mpc
bag_mpc = rosbag.Bag(bag_path_mpc)


x_list_list_mpc = []
y_list_list_mpc = []
v_x_list_list_mpc = []
v_y_list_list_mpc = []
time_sec_list_list_mpc = []
time_nsec_list_list_mpc = []

v_des_x_list_list_mpc = []
v_des_y_list_list_mpc = []
des_time_sec_list_list_mpc = []
des_time_nsec_list_list_mpc = []

abs_vel_list_list_mpc = []
abs_vel_des_list_list_mpc = []

time_mpc_sec_list_list = []
time_mpc_nsec_list_list = []
mpc_x_list_list = []
mpc_y_list_list = []


for ii in range(N_cf):
    # position trajectories
    x_list_i = []
    x_list_list_mpc.append(x_list_i)
    y_list_i = []
    y_list_list_mpc.append(y_list_i)
    # velocity trajectories
    v_x_list_i = []
    v_x_list_list_mpc.append(v_x_list_i)
    v_y_list_i = []
    v_y_list_list_mpc.append(v_y_list_i)
    # desired velocity trajectories
    v_des_x_list_i = []
    v_des_x_list_list_mpc.append(v_des_x_list_i)
    v_des_y_list_i = []
    v_des_y_list_list_mpc.append(v_des_y_list_i)
    # time
    time_sec_list_i = []
    time_sec_list_list_mpc.append(time_sec_list_i)
    time_nsec_list_i = []
    time_nsec_list_list_mpc.append(time_nsec_list_i)
    # time des vel
    des_time_sec_list_i = []
    des_time_sec_list_list_mpc.append(des_time_sec_list_i)
    des_time_nsec_list_i = []
    des_time_nsec_list_list_mpc.append(des_time_nsec_list_i)
    # time mpc
    time_mpc_sec_list_i = []
    time_mpc_sec_list_list.append(time_mpc_sec_list_i)
    time_mpc_nsec_list_i = []
    time_mpc_nsec_list_list.append(time_mpc_nsec_list_i)
    # mpc position trajectories
    mpc_x_list_i = []
    mpc_x_list_list.append(mpc_x_list_i)
    mpc_y_list_i = []
    mpc_y_list_list.append(mpc_y_list_i)


for ii in range(N_cf):
    for topic, msg, t in bag_mpc.read_messages(topics=['/cf'+ str(ii+1) +'/state']):
        # position trajectories
        x_list_list_mpc[ii].append(msg.position.x)
        y_list_list_mpc[ii].append(msg.position.y)
        # velocity trajectories
        v_x_list_list_mpc[ii].append(msg.velocity.x)
        v_y_list_list_mpc[ii].append(msg.velocity.y)
        # time
        time_sec_list_list_mpc[ii].append(t.secs)
        time_nsec_list_list_mpc[ii].append(t.nsecs)

    # putting together seconds and nanoseconds
    nsec_to_sec = [x * 10**(-9) for x in time_nsec_list_list_mpc[ii]]
    time_sec_list_list_mpc[ii] = np.add(time_sec_list_list_mpc[ii], nsec_to_sec)

for ii in range(N_cf):
    for topic, msg, t in bag_mpc.read_messages(topics=['/cf'+ str(ii+1) +'/actual_state_target']):
        # desired velocity trajectories
        v_des_x_list_list_mpc[ii].append(msg.desired_velocity.x)
        v_des_y_list_list_mpc[ii].append(msg.desired_velocity.y)
        # time
        des_time_sec_list_list_mpc[ii].append(t.secs)
        des_time_nsec_list_list_mpc[ii].append(t.nsecs)
    
    # putting together seconds and nanoseconds
    nsec_to_sec = [x * 10**(-9) for x in des_time_nsec_list_list_mpc[ii]]
    des_time_sec_list_list_mpc[ii] = np.add(des_time_sec_list_list_mpc[ii], nsec_to_sec)

for ii in range(N_cf):
    for topic, msg, t in bag_mpc.read_messages(topics=['/cf'+ str(ii+1) +'/mpc_traj']):
        # mpc optimal position trajectories
        mpc_x_list_j = []
        mpc_y_list_j = []
        for jj in range(N_mpc + 1):
            mpc_x_list_j.append(msg.x_vec[jj].desired_position.x)
            mpc_y_list_j.append(msg.x_vec[jj].desired_position.y)

        mpc_x_list_list[ii].append(mpc_x_list_j)
        mpc_y_list_list[ii].append(mpc_y_list_j)
        # time
        time_mpc_sec_list_list[ii].append(t.secs)
        time_mpc_nsec_list_list[ii].append(t.nsecs)


# +++++++++++++++++++++++++ Target Coordinates +++++++++++++++++++++++++++++++++

x_target = 2.0
y_target = 0.0

# +++++++++++++++++++++++++ Plotting trajectories ++++++++++++++++++++++++++++++

fig1,ax1 = plt.subplots()
legend_traj = []

for ii in range(N_cf):
    ax1.plot(x_list_list_mpc[ii], y_list_list_mpc[ii])
    legend_traj.append('drone_'+str(ii+1)+' trajectory')


ax1.set_xlabel('x [m]')

ax1.set_ylabel('y [m]')

ax1.set_aspect("equal")
plt.grid("minor")



# ++++++++++++++++ Plotting MPC Open Loop Trajectories +++++++++++++++++++++++


if mpc_traj_flag:


    initial_time_steps_i = []

    for ii in range(N_cf):
        N_max_time_i = len(time_mpc_sec_list_list[ii])
        initial_time_steps_i = np.arange(0,N_max_time_i,60)
        for jj in range(3):
            initial_time_steps_i = np.delete(initial_time_steps_i,-1,0)


        for jj in initial_time_steps_i:
            ax1.plot(mpc_x_list_list[ii][jj],mpc_y_list_list[ii][jj],'o--')
            legend_traj.append('mpc open loop prediction' + str(jj))


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

for ii in range(N_obs):
    ax1.plot(x_obs[ii] + r_obs[ii]*np.cos(dummy_angle),
             y_obs[ii] + r_obs[ii]*np.sin(dummy_angle), 'k')





# +++++++++++++++++++++ Plotting velocity vs. time +++++++++++++++++++++++++++



fig2,ax2 = plt.subplots()
legend_vel = []

for ii in range(N_cf):
    vel_x_list_i = np.array(v_x_list_list_mpc[ii])
    vel_y_list_i = np.array(v_y_list_list_mpc[ii])
    vel_des_x_list_i = np.array(v_des_x_list_list_mpc[ii])
    vel_des_y_list_i = np.array(v_des_y_list_list_mpc[ii])
    
    abs_vel_list_i = np.sqrt(np.add(np.square(vel_x_list_i), np.square(vel_y_list_i)))
    abs_vel_des_list_i = np.sqrt(np.add(np.square(vel_des_x_list_i), np.square(vel_des_y_list_i)))

    print('length of abs_vel_list_i is: ', len(abs_vel_list_i))

    abs_vel_list_list_mpc.append(abs_vel_list_i)
    abs_vel_des_list_list_mpc.append(abs_vel_des_list_i)



for ii in range(N_cf):
    ax2.plot(time_sec_list_list_mpc[ii], abs_vel_list_list_mpc[ii])
    ax2.plot(des_time_sec_list_list_mpc[ii], abs_vel_des_list_list_mpc[ii])
    legend_vel.append('v_'+str(ii+1))
    legend_vel.append('v_des_'+str(ii+1))

ax2.legend(legend_vel)

ax2.set_xlabel('t [s]')

ax2.set_ylabel('v [m/s]')

plt.grid("minor")

# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

fig3,ax3 = plt.subplots()
legend_vel = []

for ii in range(N_cf):
    ax3.plot(time_sec_list_list_mpc[ii], v_x_list_list_mpc[ii])
    legend_vel.append('v_x_'+str(ii+1))
    ax3.plot(time_sec_list_list_mpc[ii], v_y_list_list_mpc[ii])
    legend_vel.append('v_y_'+str(ii+1))

ax3.legend(legend_vel)

ax3.set_xlabel('t [s]')

ax3.set_ylabel('v_x, v_y [m/s]')

plt.grid("minor")

# +++++++++++++ Plotting distance of Center of Mass from Target ++++++++++++++

N_time_steps_cm_list = []

for ii in range(N_cf):
    N_time_steps_cm_list.append(len(x_list_list_mpc[ii]))

N_time_steps_cm_list.sort()

N_time_steps_cm = N_time_steps_cm_list[0]

distance_cm_list = []


x_cm_list = []
y_cm_list = []

for kk in range(N_time_steps_cm):
    x_cm_sum = 0
    y_cm_sum = 0
    for ii in range(N_cf):
        x_cm_sum += x_list_list_mpc[ii][kk]
        y_cm_sum += y_list_list_mpc[ii][kk]
    x_cm = x_cm_sum/N_cf
    y_cm = y_cm_sum/N_cf
    x_cm_list.append(x_cm)
    y_cm_list.append(y_cm)



for kk in range(N_time_steps_cm):
    dist = sqrt((x_cm_list[kk] - x_target)**2 + (y_cm_list[kk] - y_target)**2)
    distance_cm_list.append(dist)

time_sec_list_cm = time_sec_list_list_mpc[0][0:N_time_steps_cm]

fig4,ax4 = plt.subplots()

# ax4.plot(time_sec_list_cm, distance_cm_list)

ax4.set_xlabel('t [s]')
ax4.set_ylabel('d_cm [m]')


time_smooth_cm = np.linspace(time_sec_list_cm[0], time_sec_list_cm[-1], N_time_steps_cm)

ax4.plot(time_smooth_cm, distance_cm_list)


plt.grid("minor")


# ++++++++++ Plotting Velocity Trajectories - Actual vs. Desired ++++++++++++++

fig5,ax5 = plt.subplots()
legend_vel_traj = []

for ii in range(N_cf):
    ax5.plot(v_x_list_list_mpc[ii][500:1850], v_y_list_list_mpc[ii][500:1850])
    ax5.plot(v_des_x_list_list_mpc[ii][500:1850], v_des_y_list_list_mpc[ii][500:1850])

    legend_vel_traj.append('drone_'+str(ii+1)+' actual velocity trajectory')
    legend_vel_traj.append('drone_'+str(ii+1)+' desired velocity trajectory')


ax5.legend(legend_vel_traj)
ax5.set_xlabel('v_x [m]')

ax5.set_ylabel('v_y [m]')

ax5.set_aspect("equal")
plt.grid("minor")



# +++++++++++++++++++++ Plotting Velocity Error +++++++++++++++++++++++++++++++


v_x_interp_list_list_mpc = []
v_y_interp_list_list_mpc = []
v_abs_interp_list_list_mpc = []

for ii in range(N_cf):
    # velocity interp trajectories
    v_x_interp_list_i = []
    v_x_interp_list_list_mpc.append(v_x_interp_list_i)
    v_y_interp_list_i = []
    v_y_interp_list_list_mpc.append(v_y_interp_list_i)
    v_abs_interp_list_i = []
    v_abs_interp_list_list_mpc.append(v_abs_interp_list_i)

for ii in range(N_cf):
    v_x_interp_list_list_mpc[ii] = (np.interp(des_time_sec_list_list_mpc[ii],
                                        time_sec_list_list_mpc[ii], v_x_list_list_mpc[ii]))
    v_y_interp_list_list_mpc[ii] = (np.interp(des_time_sec_list_list_mpc[ii],
                                        time_sec_list_list_mpc[ii], v_y_list_list_mpc[ii]))
    v_abs_interp_list_list_mpc[ii] = (np.interp(des_time_sec_list_list_mpc[ii],
                                        time_sec_list_list_mpc[ii], abs_vel_list_list_mpc[ii]))



# Velocity error
v_error_list_list_mpc = []
for ii in range(N_cf):
    v_error_list_list_mpc.append(np.subtract(v_abs_interp_list_list_mpc[ii],
                                             abs_vel_des_list_list_mpc[ii]))



fig6,ax6 = plt.subplots()
legend_vel_err = []


for ii in range(N_cf):
    # ax6.plot(v_x_interp_list_list[ii][500:2200], v_y_interp_list_list[ii][500:2200])
    # ax6.plot(v_des_x_list_list[ii][500:2200], v_des_y_list_list[ii][500:2200])
    ax6.plot(des_time_sec_list_list_mpc[ii], v_error_list_list_mpc[ii])

    # legend_vel_traj.append('drone_'+str(ii+1)+' actual velocity trajectory')
    # legend_vel_traj.append('drone_'+str(ii+1)+' desired velocity trajectory')
    pass













plt.show(block=True)

bag_mpc.close()