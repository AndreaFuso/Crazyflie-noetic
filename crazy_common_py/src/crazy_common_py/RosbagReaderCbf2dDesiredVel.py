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

rospack = rospkg.RosPack()


bag_name = '1_drone_cbf_sim_test15.bag'
bag_path = rospack.get_path('crazyCmd') + '/data/output/RosbagsPietro/' + bag_name
bag = rosbag.Bag(bag_path)


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

abs_vel_list_list = []
abs_vel_des_list_list = []

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
    for topic, msg, t in bag.read_messages(topics=['/cf'+ str(ii+1) +'/state']):
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
    for topic, msg, t in bag.read_messages(topics=['/cf'+ str(ii+1) +'/actual_state_target']):
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
    for topic, msg, t in bag.read_messages(topics=['/cf'+ str(ii+1) +'/cbf_function']):
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

# +++++++++++++++++++++++++ Target Coordinates +++++++++++++++++++++++++++++++++

x_target = 2.0
y_target = 0.0

# +++++++++++++++++++++++++ Plotting trajectories ++++++++++++++++++++++++++++++

fig1,ax1 = plt.subplots()
legend_traj = []

for ii in range(N_cf):
    ax1.plot(x_list_list_cbf[ii], y_list_list_cbf[ii])
    legend_traj.append('drone_'+str(ii+1)+' trajectory')


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


fig2,ax2 = plt.subplots()
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

    abs_vel_list_list.append(abs_vel_list_i)
    abs_vel_des_list_list.append(abs_vel_des_list_i)



for ii in range(N_cf):
    ax2.plot(time_sec_list_list_cbf[ii], abs_vel_list_list[ii])
    ax2.plot(des_time_sec_list_list_cbf[ii], abs_vel_des_list_list[ii])
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
    ax3.plot(time_sec_list_list_cbf[ii], v_x_list_list_cbf[ii])
    legend_vel.append('v_x_'+str(ii+1))
    ax3.plot(time_sec_list_list_cbf[ii], v_y_list_list_cbf[ii])
    legend_vel.append('v_y_'+str(ii+1))

ax3.legend(legend_vel)

ax3.set_xlabel('t [s]')

ax3.set_ylabel('v_x, v_y [m/s]')

plt.grid("minor")


# ++++++++++ Plotting Velocity Trajectories - Actual vs. Desired ++++++++++++++

fig5,ax5 = plt.subplots()
legend_vel_traj = []

for ii in range(N_cf):
    ax5.plot(v_x_list_list_cbf[ii][450:2200], v_y_list_list_cbf[ii][450:2200])
    ax5.plot(v_des_x_list_list_cbf[ii][450:2200], v_des_y_list_list_cbf[ii][450:2200])

    legend_vel_traj.append('drone_'+str(ii+1)+' actual velocity trajectory')
    legend_vel_traj.append('drone_'+str(ii+1)+' desired velocity trajectory')


ax5.legend(legend_vel_traj)
ax5.set_xlabel('v_x [m]')

ax5.set_ylabel('v_y [m]')

ax5.set_aspect("equal")
plt.grid("minor")



# +++++++++++++++++++++ Plotting Velocity Error +++++++++++++++++++++++++++++++


v_x_interp_list_list = []
v_y_interp_list_list = []
v_abs_interp_list_list = []
x_interp_list_list = []
y_interp_list_list = []

for ii in range(N_cf):
    # velocity interp trajectories
    v_x_interp_list_i = []
    v_x_interp_list_list.append(v_x_interp_list_i)
    v_y_interp_list_i = []
    v_y_interp_list_list.append(v_y_interp_list_i)
    v_abs_interp_list_i = []
    v_abs_interp_list_list.append(v_abs_interp_list_i)
    # position interp trajectories
    x_interp_list_i = []
    x_interp_list_list.append(v_x_interp_list_i)
    y_interp_list_i = []
    y_interp_list_list.append(v_y_interp_list_i)

for ii in range(N_cf):
    v_x_interp_list_list[ii] = (np.interp(des_time_sec_list_list_cbf[ii],
                            time_sec_list_list_cbf[ii], v_x_list_list_cbf[ii]))
    v_y_interp_list_list[ii] = (np.interp(des_time_sec_list_list_cbf[ii],
                            time_sec_list_list_cbf[ii], v_y_list_list_cbf[ii]))
    v_abs_interp_list_list[ii] = (np.interp(des_time_sec_list_list_cbf[ii],
                            time_sec_list_list_cbf[ii], abs_vel_list_list[ii]))
    x_interp_list_list[ii] = (np.interp(des_time_sec_list_list_cbf[ii],
                            time_sec_list_list_cbf[ii], x_list_list_cbf[ii]))
    y_interp_list_list[ii] = (np.interp(des_time_sec_list_list_cbf[ii],
                            time_sec_list_list_cbf[ii], y_list_list_cbf[ii]))




# Velocity error
v_error_list_list = []
for ii in range(N_cf):
    v_error_list_list.append(np.subtract(v_abs_interp_list_list[ii],
                                         abs_vel_des_list_list[ii]))
    pass


fig6,ax6 = plt.subplots()
legend_vel_err = []


for ii in range(N_cf):
    # ax6.plot(v_x_interp_list_list[ii][500:2200], v_y_interp_list_list[ii][500:2200])
    # ax6.plot(v_des_x_list_list[ii][500:2200], v_des_y_list_list[ii][500:2200])
    ax6.plot(des_time_sec_list_list_cbf[ii], v_error_list_list[ii])

    # legend_vel_traj.append('drone_'+str(ii+1)+' actual velocity trajectory')
    # legend_vel_traj.append('drone_'+str(ii+1)+' desired velocity trajectory')
    pass

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
                                des_time_sec_list_list_cbf[ii], abs_vel_des_list_list[ii]))






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


plt.show(block=True)

bag.close()