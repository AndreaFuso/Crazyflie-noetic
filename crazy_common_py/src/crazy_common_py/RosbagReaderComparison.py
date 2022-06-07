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
mpc_traj_flag = False

rospack = rospkg.RosPack()

# +++++++++++++++++++++ Extracting data about cbf +++++++++++++++++++++++++++++++++

bag_name_cbf = '1_drone_cbf_sim_test15.bag'
bag_path_cbf = rospack.get_path('crazyCmd') + '/data/output/RosbagsPietro/' + bag_name_cbf
bag_cbf = rosbag.Bag(bag_path_cbf)


x_list_list_cbf = []
y_list_list_cbf = []
v_x_list_list_cbf = []
v_y_list_list_cbf = []
time_sec_list_list_cbf = []
time_nsec_list_list_cbf = []

v_des_x_list_list_cbf = []
v_des_y_list_list_cbf = []
des_time_sec_list_list_cbf = []
des_time_nsec_list_list_cbf = []

abs_vel_list_list_cbf = []
abs_vel_des_list_list_cbf = []

time_mpc_sec_list_list = []
time_mpc_nsec_list_list = []
mpc_x_list_list = []
mpc_y_list_list = []

h_fun_list_list = []
h_p_list_list = []
min_alpha_h_list_list = []

time_h_sec_list_list = []
time_h_nsec_list_list = []


for ii in range(N_cf):
    # position trajectories
    x_list_i = []
    x_list_list_cbf.append(x_list_i)
    y_list_i = []
    y_list_list_cbf.append(y_list_i)
    # velocity trajectories
    v_x_list_i = []
    v_x_list_list_cbf.append(v_x_list_i)
    v_y_list_i = []
    v_y_list_list_cbf.append(v_y_list_i)
    # desired velocity trajectories
    v_des_x_list_i = []
    v_des_x_list_list_cbf.append(v_des_x_list_i)
    v_des_y_list_i = []
    v_des_y_list_list_cbf.append(v_des_y_list_i)
    # time
    time_sec_list_i = []
    time_sec_list_list_cbf.append(time_sec_list_i)
    time_nsec_list_i = []
    time_nsec_list_list_cbf.append(time_nsec_list_i)
    # time des vel
    des_time_sec_list_i = []
    des_time_sec_list_list_cbf.append(des_time_sec_list_i)
    des_time_nsec_list_i = []
    des_time_nsec_list_list_cbf.append(des_time_nsec_list_i)
    # h_fun history
    h_fun_list_i = []
    h_fun_list_list.append(h_fun_list_i)
    # h_p history
    h_p_list_i = []
    h_p_list_list.append(h_p_list_i)
    # alpha_h history
    alpha_h_list_i = []
    min_alpha_h_list_list.append(alpha_h_list_i)
    # time h
    time_h_sec_list_i = []
    time_h_sec_list_list.append(time_h_sec_list_i)
    time_h_nsec_list_i = []
    time_h_nsec_list_list.append(time_h_nsec_list_i)



for ii in range(N_cf):
    for topic, msg, t in bag_cbf.read_messages(topics=['/cf'+ str(ii+1) +'/state']):
        # position trajectories
        x_list_list_cbf[ii].append(msg.position.x)
        y_list_list_cbf[ii].append(msg.position.y)
        # velocity trajectories
        v_x_list_list_cbf[ii].append(msg.velocity.x)
        v_y_list_list_cbf[ii].append(msg.velocity.y)
        # time
        time_sec_list_list_cbf[ii].append(t.secs)
        time_nsec_list_list_cbf[ii].append(t.nsecs)

    # putting together seconds and nanoseconds
    nsec_to_sec = [x * 10**(-9) for x in time_nsec_list_list_cbf[ii]]
    time_sec_list_list_cbf[ii] = np.add(time_sec_list_list_cbf[ii], nsec_to_sec)


    print('time_sec_list_list[ii] is: ', time_sec_list_list_cbf[ii])
    print('length time_sec_list_list[ii] is: ', len(time_sec_list_list_cbf[ii]))

for ii in range(N_cf):
    for topic, msg, t in bag_cbf.read_messages(topics=['/cf'+ str(ii+1) +'/actual_state_target']):
    # for topic, msg, t in bag.read_messages(topics=['/cf'+ str(ii+1) +'/mpc_velocity']):
        # desired velocity trajectories
        v_des_x_list_list_cbf[ii].append(msg.desired_velocity.x)
        v_des_y_list_list_cbf[ii].append(msg.desired_velocity.y)
        # time
        des_time_sec_list_list_cbf[ii].append(t.secs)
        des_time_nsec_list_list_cbf[ii].append(t.nsecs)

    # putting together seconds and nanoseconds
    nsec_to_sec = [x * 10**(-9) for x in des_time_nsec_list_list_cbf[ii]]
    des_time_sec_list_list_cbf[ii] = np.add(des_time_sec_list_list_cbf[ii], nsec_to_sec)

for ii in range(N_cf):
    for topic, msg, t in bag_cbf.read_messages(topics=['/cf'+ str(ii+1) +'/cbf_function']):
        # desired velocity trajectories
        h_fun_list_list[ii].append(msg.desired_position.z)
        h_p_list_list[ii].append(msg.desired_position.y)
        min_alpha_h_list_list[ii].append(msg.desired_position.x)
        # time
        time_h_sec_list_list[ii].append(t.secs)
        time_h_nsec_list_list[ii].append(t.nsecs)

    # putting together seconds and nanoseconds
    nsec_to_sec = [x * 10**(-9) for x in time_h_nsec_list_list[ii]]
    time_h_sec_list_list[ii] = np.add(time_h_sec_list_list[ii], nsec_to_sec)


# +++++++++++++++++++++ Extracting data about mpc +++++++++++++++++++++++++++++++++

bag_name_mpc = '1_drone_mpc_sim_test2.bag'
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

abs_vel_list_list_cbf = []
abs_vel_des_list_list_cbf = []


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
    ax1.plot(x_list_list_cbf[ii], y_list_list_cbf[ii])
    legend_traj.append('drone_'+str(ii+1) + ' mpc trajectory')
    legend_traj.append('drone_'+str(ii+1) + ' cbf trajectory')


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
    vel_x_list_i = np.array(v_x_list_list_cbf[ii])
    vel_y_list_i = np.array(v_y_list_list_cbf[ii])
    # Desired velocities fed to velocity controller
    vel_des_x_list_i = np.array(v_des_x_list_list_cbf[ii])
    vel_des_y_list_i = np.array(v_des_y_list_list_cbf[ii])
    # Absolute actual velocity
    abs_vel_list_i = np.sqrt(np.add(np.square(vel_x_list_i), np.square(vel_y_list_i)))
    # Absolute desired velocity
    abs_vel_des_list_i = np.sqrt(np.add(np.square(vel_des_x_list_i), np.square(vel_des_y_list_i)))

    abs_vel_list_list_cbf.append(abs_vel_list_i)
    abs_vel_des_list_list_cbf.append(abs_vel_des_list_i)


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
    ax2.plot(time_sec_list_list_cbf[ii], abs_vel_list_list_cbf[ii])
    ax2.plot(des_time_sec_list_list_cbf[ii], abs_vel_des_list_list_cbf[ii])
    legend_vel.append('v_' + str(ii+1) + ' cbf')
    legend_vel.append('v_des_' + str(ii+1) + ' cbf')

ax2.legend(legend_vel)

ax2.set_xlabel('t [s]')
ax2.set_ylabel('v [m/s]')
ax2.set_title('CBF')

ax2.grid("minor")

# fig10,ax10 = plt.subplots()
legend_vel_mpc = []

for ii in range(N_cf):
    ax10.plot(time_sec_list_list_mpc[ii], abs_vel_list_list_mpc[ii])
    ax10.plot(des_time_sec_list_list_mpc[ii], abs_vel_des_list_list_mpc[ii])
    legend_vel_mpc.append('v_' + str(ii+1) + ' mpc')
    legend_vel_mpc.append('v_des_' + str(ii+1) + ' mpc')

ax10.legend(legend_vel_mpc)

ax10.set_xlabel('t [s]')
ax10.set_ylabel('v [m/s]')
ax10.set_title('MPC')

ax10.grid("minor")


fig11,ax11 = plt.subplots()
legend_vel_comp = []

for ii in range(N_cf):
    ax11.plot(time_sec_list_list_mpc[ii], abs_vel_list_list_mpc[ii])
    ax11.plot(time_sec_list_list_cbf[ii], abs_vel_list_list_cbf[ii])
    legend_vel_comp.append('v_' + str(ii+1) + ' mpc')
    legend_vel_comp.append('v_' + str(ii+1) + ' cbf')

ax11.legend(legend_vel_comp)

ax11.set_xlabel('t [s]')
ax11.set_ylabel('v [m/s]')


ax11.grid("minor")

# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

fig3,ax3 = plt.subplots()
legend_vel = []

for ii in range(N_cf):
    ax3.plot(time_sec_list_list_cbf[ii], v_x_list_list_cbf[ii])
    legend_vel.append('v_x_' + str(ii+1) + ' cbf')
    ax3.plot(time_sec_list_list_cbf[ii], v_y_list_list_cbf[ii])
    legend_vel.append('v_y_' + str(ii+1) + ' cbf')

ax3.legend(legend_vel)

ax3.set_xlabel('t [s]')

ax3.set_ylabel('v_x, v_y [m/s]')

fig13,ax13 = plt.subplots()
legend_vel = []

for ii in range(N_cf):
    ax13.plot(time_sec_list_list_mpc[ii], v_x_list_list_mpc[ii])
    legend_vel.append('v_x_' + str(ii+1) + ' mpc')
    ax13.plot(time_sec_list_list_mpc[ii], v_y_list_list_mpc[ii])
    legend_vel.append('v_y_' + str(ii+1) + ' mpc')

ax13.legend(legend_vel)

ax13.set_xlabel('t [s]')

ax13.set_ylabel('v_x, v_y [m/s]')

plt.grid("minor")


# ++++++++++ Plotting Velocity Trajectories - Actual vs. Desired ++++++++++++++

fig5,(ax14,ax5) = plt.subplots(1,2)
legend_vel_traj = []

for ii in range(N_cf):
    ax5.plot(v_x_list_list_cbf[ii][200:2200], v_y_list_list_cbf[ii][200:2200])
    ax5.plot(v_des_x_list_list_cbf[ii][200:2200], v_des_y_list_list_cbf[ii][200:2200])

    legend_vel_traj.append('drone_'+str(ii+1)+' actual velocity trajectory')
    legend_vel_traj.append('drone_'+str(ii+1)+' desired velocity trajectory')


ax5.legend(legend_vel_traj)
ax5.set_xlabel('v_x [m]')

ax5.set_ylabel('v_y [m]')

ax5.set_aspect("equal")

ax5.set_title("CBF")

ax5.grid("minor")

# fig14,ax14 = plt.subplots()
legend_vel_traj = []

for ii in range(N_cf):
    ax14.plot(v_x_list_list_mpc[ii][500:1850], v_y_list_list_mpc[ii][500:1850])
    ax14.plot(v_des_x_list_list_mpc[ii][500:1850], v_des_y_list_list_mpc[ii][500:1850])

    legend_vel_traj.append('drone_' + str(ii+1) + ' actual velocity trajectory')
    legend_vel_traj.append('drone_' + str(ii+1) + ' desired velocity trajectory')

ax14.legend(legend_vel_traj)
ax14.set_xlabel('v_x [m]')

ax14.set_ylabel('v_y [m]')

ax14.set_aspect("equal")

ax14.set_title("MPC")

ax14.grid("minor")



# +++++++++++++++++++++ Plotting Velocity Error +++++++++++++++++++++++++++++++


v_x_interp_list_list_cbf = []
v_y_interp_list_list_cbf = []
v_abs_interp_list_list_cbf = []
x_interp_list_list_cbf = []
y_interp_list_list_cbf = []

for ii in range(N_cf):
    # velocity interp trajectories
    v_x_interp_list_i = []
    v_x_interp_list_list_cbf.append(v_x_interp_list_i)
    v_y_interp_list_i = []
    v_y_interp_list_list_cbf.append(v_y_interp_list_i)
    v_abs_interp_list_i = []
    v_abs_interp_list_list_cbf.append(v_abs_interp_list_i)
    # position interp trajectories
    x_interp_list_i = []
    x_interp_list_list_cbf.append(v_x_interp_list_i)
    y_interp_list_i = []
    y_interp_list_list_cbf.append(v_y_interp_list_i)

for ii in range(N_cf):
    v_x_interp_list_list_cbf[ii] = (np.interp(des_time_sec_list_list_cbf[ii],
                            time_sec_list_list_cbf[ii], v_x_list_list_cbf[ii]))
    v_y_interp_list_list_cbf[ii] = (np.interp(des_time_sec_list_list_cbf[ii],
                            time_sec_list_list_cbf[ii], v_y_list_list_cbf[ii]))
    v_abs_interp_list_list_cbf[ii] = (np.interp(des_time_sec_list_list_cbf[ii],
                            time_sec_list_list_cbf[ii], abs_vel_list_list_cbf[ii]))
    x_interp_list_list_cbf[ii] = (np.interp(des_time_sec_list_list_cbf[ii],
                            time_sec_list_list_cbf[ii], x_list_list_cbf[ii]))
    y_interp_list_list_cbf[ii] = (np.interp(des_time_sec_list_list_cbf[ii],
                            time_sec_list_list_cbf[ii], y_list_list_cbf[ii]))


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
    v_error = np.absolute(np.subtract(v_abs_interp_list_list_mpc[ii],
              abs_vel_des_list_list_mpc[ii]))
    v_error_list_list_mpc.append(v_error)


# Velocity error
v_error_list_list_cbf = []
for ii in range(N_cf):
    v_error = np.absolute(np.subtract(v_abs_interp_list_list_cbf[ii],
              abs_vel_des_list_list_cbf[ii]))
    v_error_list_list_cbf.append(v_error)


fig6,ax6 = plt.subplots()
legend_vel_err = []


for ii in range(N_cf):
    ax6.plot(des_time_sec_list_list_mpc[ii], v_error_list_list_mpc[ii])
    ax6.plot(des_time_sec_list_list_cbf[ii], v_error_list_list_cbf[ii])
    legend_vel_err.append('drone_' + str(ii+1) + ' velocity error mpc')
    legend_vel_err.append('drone_' + str(ii+1) + ' velocity error cbf')

ax6.legend(legend_vel_err)
ax6.set_xlabel('time [s]')

ax6.set_ylabel('e [m/s]')


ax6.grid("minor")
# +++++++++++++ Interpolating v_des on time_sec_list_list ++++++++++++++


v_des_x_interp_list_list = []
v_des_y_interp_list_list = []
v_abs_des_interp_list_list = []

for ii in range(N_cf):
    # velocity interp trajectories
    v_des_x_interp_list_i = []
    v_des_x_interp_list_list.append(v_des_x_interp_list_i)
    v_des_y_interp_list_i = []
    v_des_y_interp_list_list.append(v_des_y_interp_list_i)
    v_des_abs_interp_list_i = []
    v_abs_des_interp_list_list.append(v_des_abs_interp_list_i)

for ii in range(N_cf):
    v_des_x_interp_list_list[ii] = (np.interp(time_sec_list_list_cbf[ii],
                    des_time_sec_list_list_cbf[ii], v_des_x_list_list_cbf[ii]))
    v_des_y_interp_list_list[ii] = (np.interp(time_sec_list_list_cbf[ii],
                    des_time_sec_list_list_cbf[ii], v_des_y_list_list_cbf[ii]))
    v_abs_des_interp_list_list[ii] = (np.interp(time_sec_list_list_cbf[ii],
                    des_time_sec_list_list_cbf[ii], abs_vel_des_list_list_cbf[ii]))






# ++++++++++++++++++++ Plotting h cbf function ++++++++++++++++++++++++++++++

fig7,ax7 = plt.subplots()


for ii in range(N_cf):
    ax7.plot(time_h_sec_list_list[ii], h_fun_list_list[ii])


ax7.set_xlabel('time [s]')

ax7.set_ylabel('h')
# ax7.set_title('h function')







# ++++++++++++++++++++++++ Plotting constraint ++++++++++++++++++++++++++++++

fig9,ax9 = plt.subplots()

legend_h_p = []

for ii in range(N_cf):
    ax9.plot(time_h_sec_list_list[ii], h_p_list_list[ii])
    ax9.plot(time_h_sec_list_list[ii], min_alpha_h_list_list[ii], '--')
    legend_h_p.append('h_p')
    legend_h_p.append('-alpha*h')



ax9.set_xlabel('time [s]')

ax9.set_ylabel('h_p vs -alpha*h')
# ax9.set_title('constraint')

ax9.legend(legend_h_p)

plt.grid("minor")



# +++++++++++++ Plotting distance of Center of Mass from Target ++++++++++++++



distance_cm_list_mpc = []
N_time_steps_cm_mpc = len(x_list_list_mpc[0])
print('N_time_steps_cm_mpc is: ', N_time_steps_cm_mpc)
legend_distance_cm = []
x_cm_list_mpc = []
y_cm_list_mpc = []

for kk in range(N_time_steps_cm_mpc):
    x_cm_sum = 0
    y_cm_sum = 0
    for ii in range(N_cf):
        x_cm_sum += x_list_list_mpc[ii][kk]
        y_cm_sum += y_list_list_mpc[ii][kk]
    x_cm = x_cm_sum/N_cf
    y_cm = y_cm_sum/N_cf
    x_cm_list_mpc.append(x_cm)
    y_cm_list_mpc.append(y_cm)



for kk in range(N_time_steps_cm_mpc):
    dist = sqrt((x_cm_list_mpc[kk] - x_target)**2 + (y_cm_list_mpc[kk] - y_target)**2)
    distance_cm_list_mpc.append(dist)

legend_distance_cm.append('absolute distance mpc')
fig15,ax15 = plt.subplots()


ax15.set_xlabel('t [s]')
ax15.set_ylabel('d [m]')


ax15.plot(time_sec_list_list_mpc[0], distance_cm_list_mpc)



distance_cm_list_cbf = []
N_time_steps_cm_cbf = len(x_list_list_cbf[0])
print('N_time_steps_cm_mpc is: ', N_time_steps_cm_cbf)
legend_distance_cbf = []
x_cm_list_cbf = []
y_cm_list_cbf = []

for kk in range(N_time_steps_cm_cbf):
    x_cm_sum = 0
    y_cm_sum = 0
    for ii in range(N_cf):
        x_cm_sum += x_list_list_cbf[ii][kk]
        y_cm_sum += y_list_list_cbf[ii][kk]
    x_cm = x_cm_sum/N_cf
    y_cm = y_cm_sum/N_cf
    x_cm_list_cbf.append(x_cm)
    y_cm_list_cbf.append(y_cm)



for kk in range(N_time_steps_cm_cbf):
    dist = sqrt((x_cm_list_cbf[kk] - x_target)**2 + (y_cm_list_cbf[kk] - y_target)**2)
    distance_cm_list_cbf.append(dist)

legend_distance_cm.append('absolute distance cbf')


ax15.set_xlabel('t [s]')
ax15.set_ylabel('d [m]')


ax15.plot(time_sec_list_list_cbf[0], distance_cm_list_cbf)

ax15.legend(legend_distance_cm)




plt.grid("minor")




# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++




plt.show(block=True)

bag_cbf.close()