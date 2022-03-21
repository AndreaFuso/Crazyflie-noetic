from cmath import sqrt
from tkinter import N
import rosbag
import rospkg
import matplotlib.pyplot as plt
import numpy as np

from crazy_common_py.common_functions import rad2deg

N_cf = 6
N_obs = 3

rospack = rospkg.RosPack()

bag_name = '6_drones_cbf_coll_avoid_centralized.bag'
bag_path = rospack.get_path('crazyCmd') + '/data/output/Rosbags/' + bag_name
bag = rosbag.Bag(bag_path)


x_list_list = []
y_list_list = []
v_x_list_list = []
v_y_list_list = []
time_sec_list_list = []
time_nsec_list_list = []
abs_vel_list_list = []


for ii in range(N_cf):
    # position trajectories
    x_list_i = []
    x_list_list.append(x_list_i)
    y_list_i = []
    y_list_list.append(y_list_i)
    # velocity trajectories
    v_x_list_i = []
    v_x_list_list.append(v_x_list_i)
    v_y_list_i = []
    v_y_list_list.append(v_y_list_i)
    # time
    time_sec_list_i = []
    time_sec_list_list.append(time_sec_list_i)
    time_nsec_list_i = []
    time_nsec_list_list.append(time_nsec_list_i)
    # # absolute velocity of drones
    # abs_vel_list_i = []
    # abs_vel_list_list.append(abs_vel_list_i)


for ii in range(N_cf):
    for topic, msg, t in bag.read_messages(topics=['/cf'+ str(ii+1) +'/state']):
        # position trajectories
        x_list_list[ii].append(msg.position.x)
        y_list_list[ii].append(msg.position.y)
        # # velocity trajectories
        v_x_list_list[ii].append(msg.velocity.x)
        v_y_list_list[ii].append(msg.velocity.y)
        # time
        time_sec_list_list[ii].append(t.secs)
        time_nsec_list_list[ii].append(t.nsecs)


# +++++++++++++++++++++++++ Plotting trajectories ++++++++++++++++++++++++++++++

fig1,ax1 = plt.subplots()
legend_traj = []

for ii in range(N_cf):
    ax1.plot(x_list_list[ii], y_list_list[ii])
    legend_traj.append('drone_'+str(ii+1))


ax1.set_xlabel('x [m]')

ax1.set_ylabel('y [m]')

ax1.set_aspect("equal")
plt.grid("minor")



# +++++++++++++++++ Plotting Center of Mass trajectory +++++++++++++++++++++++

N_time_steps_list = []

for ii in range(N_cf):
    N_time_steps_list.append(len(x_list_list[ii]))

N_time_steps_list.sort()

N_time_steps = N_time_steps_list[0]

x_cm_list = []
y_cm_list = []

for kk in range(N_time_steps):
    x_cm_sum = 0
    y_cm_sum = 0
    for ii in range(N_cf):
        x_cm_sum += x_list_list[ii][kk]
        y_cm_sum += y_list_list[ii][kk]
    x_cm = x_cm_sum/N_cf
    y_cm = y_cm_sum/N_cf
    x_cm_list.append(x_cm)
    y_cm_list.append(y_cm)


ax1.plot(x_cm_list, y_cm_list)
legend_traj.append('center of mass')


ax1.legend(legend_traj)


# +++++++++++++++++++++++++ Plotting Obstacles +++++++++++++++++++++++++++++++


dummy_angle = np.linspace(0,2*np.pi,100)

x_obs = [2.0, 1.0, 2.0]
y_obs = [2.5, 2.5, 4.0]
r_obs = [0.3, 0.2, 0.4]

for ii in range(N_obs):
    ax1.plot(x_obs[ii] + r_obs[ii]*np.cos(dummy_angle),
             y_obs[ii] + r_obs[ii]*np.sin(dummy_angle), 'k')

# +++++++++++++++++++++ Plotting velocity vs. time +++++++++++++++++++++++++++


fig2,ax2 = plt.subplots()
legend_vel = []

for ii in range(N_cf):
    vel_x_list_i = np.array(v_x_list_list[ii])
    vel_y_list_i = np.array(v_y_list_list[ii])
    print('length of vel_x_list_' + str(ii) + ' is: ' , len(vel_x_list_i))
    print('length of vel_y_list_' + str(ii) + ' is: ', len(vel_y_list_i))
    
    abs_vel_list_i = np.sqrt(np.add(np.square(vel_x_list_i), np.square(vel_y_list_i)))

    print('length of abs_vel_list_i is: ', len(abs_vel_list_i))

    abs_vel_list_list.append(abs_vel_list_i)




for ii in range(N_cf):
    ax2.plot(time_sec_list_list[ii], abs_vel_list_list[ii])
    legend_vel.append('v_'+str(ii+1))

ax2.legend(legend_vel)

ax2.set_xlabel('t [s]')

ax2.set_ylabel('v [m/s]')

plt.grid("minor")

# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

fig3,ax3 = plt.subplots()
legend_vel = []


for ii in range(N_cf):
    ax3.plot(time_sec_list_list[ii], v_x_list_list[ii])
    legend_vel.append('v_x_'+str(ii+1))
    ax3.plot(time_sec_list_list[ii], v_y_list_list[ii])
    legend_vel.append('v_y_'+str(ii+1))

ax3.legend(legend_vel)

ax3.set_xlabel('t [s]')

ax3.set_ylabel('v_x, v_y [m/s]')

plt.grid("minor")

# +++++++++++++ Plotting distance of Center of Mass from Target ++++++++++++++

x_target = 3.0
y_target = 6.0

distance_cm_list = []

for kk in range(N_time_steps):
    dist = sqrt((x_cm_list[kk] - x_target)**2 + (y_cm_list[kk] - y_target)**2)
    distance_cm_list.append(dist)

time_sec_list_cm = time_sec_list_list[0][0:N_time_steps]

fig4,ax4 = plt.subplots()

ax4.plot(time_sec_list_cm, distance_cm_list)

ax4.set_xlabel('t [s]')
ax4.set_ylabel('d_cm [m]')


time_smooth = np.linspace(time_sec_list_cm[0], time_sec_list_cm[-1], N_time_steps)

ax4.plot(time_smooth, distance_cm_list)


plt.grid("minor")


plt.show(block=True)

bag.close()