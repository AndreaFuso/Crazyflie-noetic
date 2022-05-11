#! /usr/bin/env python3

from cmath import sqrt
from tkinter import N
import rosbag
import rospkg
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits import mplot3d
from crazy_common_py.default_topics import DEFAULT_FLOCK_TOPIC, DEFAULT_CF_STATE_TOPIC, \
     DEFAULT_100Hz_PACE_TOPIC, DEFAULT_MOTOR_CMD_TOPIC, DEFAULT_ACTUAL_DESTINATION_TOPIC


from crazy_common_py.common_functions import rad2deg

rospack = rospkg.RosPack()

bag_name = '2_drones_takeoff.bag'
bag_path = rospack.get_path('crazyCmd') + '/data/output/RosbagsPietro/' + bag_name
bag = rosbag.Bag(bag_path)

N_cf = 2

# Position trajectories
x_list_list = []
y_list_list = []
z_list_list = []

# Velocity trajectories
vx_list_list = []
vy_list_list = []
vz_list_list = []


# Attitude trajectories
roll_list_list = []
pitch_list_list = []
yaw_list_list = []

# Time
time_sec_list_list = []
time_nsec_list_list = []

time_mpc_sec_list_list = []
time_mpc_nsec_list_list = []


for ii in range(N_cf):
    # Position trajectories
    x_list_i = []
    x_list_list.append(x_list_i)
    y_list_i = []
    y_list_list.append(y_list_i)
    z_list_i = []
    z_list_list.append(z_list_i)

    # Velocity trajectories
    vx_list_i = []
    vx_list_list.append(vx_list_i)
    vy_list_i = []
    vy_list_list.append(vy_list_i)
    vz_list_i = []
    vz_list_list.append(vz_list_i)
    abs_vel_list_list = []

    # Attitude trajectories
    roll_list_i = []
    roll_list_list.append(roll_list_i)
    pitch_list_i = []
    pitch_list_list.append(pitch_list_i)
    yaw_list_i = []
    yaw_list_list.append(yaw_list_i)

    # time
    time_sec_list_i = []
    time_sec_list_list.append(time_sec_list_i)
    time_nsec_list_i = []
    time_nsec_list_list.append(time_nsec_list_i)

# Desired attitude dictionaries of lists
desired_roll_list = dict()
desired_pitch_list = dict()
desired_yaw_list = dict()
desired_thrust_list = dict()

# Extracting data that where published to relevant topics

topics_list = []
topics_list.append('/swarm/states')
cf_names = []

for ii in range(N_cf):
    cf_name = 'cf' + str(ii+1)
    cf_names.append(cf_name)
    topics_list.append('/' + cf_name + '/' + DEFAULT_MOTOR_CMD_TOPIC)
    topics_list.append('/' + cf_name + '/' + DEFAULT_ACTUAL_DESTINATION_TOPIC)
    topics_list.append('/' + cf_name + '/state')


for topic, msg, t in bag.read_messages(topics=topics_list):

    if topic == '/swarm/states':
        for ii in range(N_cf):
            # Position trajectories
            x_list_list[ii].append(msg.states[ii].position.x)
            y_list_list[ii].append(msg.states[ii].position.y)
            z_list_list[ii].append(msg.states[ii].position.z)

            # Velocity trajectories
            vx_list_list[ii].append(msg.states[ii].velocity.x)
            vy_list_list[ii].append(msg.states[ii].velocity.y)
            vz_list_list[ii].append(msg.states[ii].velocity.z)

            # Attitude trajectories
            roll_list_list[ii].append(rad2deg(msg.states[ii].orientation.roll))
            pitch_list_list[ii].append(rad2deg(msg.states[ii].orientation.pitch))
            yaw_list_list[ii].append(rad2deg(msg.states[ii].orientation.yaw))

            # time
            time_sec_list_list[ii].append(t.secs)
            time_nsec_list_list[ii].append(t.nsecs)




    #     # print('msg.states[ii].position.x is: ', msg.states[0].position.x)
        # print('x_list_list[ii] is: ', x_list_list[0])

    # if topic == '/cf1/state':
    #     # Position trajectories
    #     x_list_list[0].append(msg.position.x)
    #     y_list_list[0].append(msg.position.y)
    #     z_list_list[0].append(msg.position.z)

    #     # Velocity trajectories
    #     vx_list_list[0].append(msg.velocity.x)
    #     vy_list_list[0].append(msg.velocity.y)
    #     vz_list_list[0].append(msg.velocity.z)

    #     # Attitude trajectories
    #     roll_list_list[0].append(rad2deg(msg.orientation.roll))
    #     pitch_list_list[0].append(rad2deg(msg.orientation.pitch))
    #     yaw_list_list[0].append(rad2deg(msg.orientation.yaw))

    #     # time
    #     time_sec_list_list[0].append(t.secs)
    #     time_nsec_list_list[0].append(t.nsecs)
    #     pass

    # if topic == '/cf2/state':
    #     # Position trajectories
    #     x_list_list[1].append(msg.position.x)
    #     y_list_list[1].append(msg.position.y)
    #     z_list_list[1].append(msg.position.z)

    #     # Velocity trajectories
    #     vx_list_list[1].append(msg.velocity.x)
    #     vy_list_list[1].append(msg.velocity.y)
    #     vz_list_list[1].append(msg.velocity.z)

    #     # Attitude trajectories
    #     roll_list_list[1].append(rad2deg(msg.orientation.roll))
    #     pitch_list_list[1].append(rad2deg(msg.orientation.pitch))
    #     yaw_list_list[1].append(rad2deg(msg.orientation.yaw))

    #     # time
    #     time_sec_list_list[1].append(t.secs)
    #     time_nsec_list_list[1].append(t.nsecs)
    #     pass


    # for cf_name in cf_names:
    #     if topic == '/' + cf_name + '/' + DEFAULT_MOTOR_CMD_TOPIC:
    #         desired_roll_list[cf_name] = msg.desired_attitude.roll
    #         desired_pitch_list[cf_name] = msg.desired_attitude.pitch
    #         desired_yaw_list[cf_name] = msg.desired_attitude.yaw
    #         desired_thrust_list[cf_name] = msg.desired_thrust
    #         print('desired_thrust is: ', desired_thrust_list[cf_name])

    #         pass

    # for cf_name in cf_names:
    #     if topic == '/' + cf_name + '/' + DEFAULT_ACTUAL_DESTINATION_TOPIC:
    #         pass


    # print('time_sec_list_list is: ', time_sec_list_list)

    # print('time_nsec_list_list is: ', time_nsec_list_list)



# Adding an offset of 1 m to the y coordinate of the 2nd drone


for ii in range(N_cf):

    y_list_list[ii] = [x + ii*1 for x in y_list_list[ii]]










# +++++++++++++++++++++ Plotting Positions vs. time +++++++++++++++++++++++++++


fig1,ax1 = plt.subplots()
plt.title('Positions vs. Time')
legend_pos = []

N_time_steps_pos_list = []

for ii in range(N_cf):
    N_time_steps_pos_list.append(len(x_list_list[ii]))

time_smooth = []

for ii in range(N_cf):
    time_smooth_i = []
    time_smooth.append(time_smooth_i)


for ii in range(N_cf):
    time_smooth_i_array = np.linspace(time_sec_list_list[ii][0], 
                    time_sec_list_list[ii][-1], N_time_steps_pos_list[ii])
    time_smooth_i_list = time_smooth_i_array.tolist()
    time_smooth[ii] = time_smooth_i_list

# print(time_smooth)

    # print('time_sec_list_list[ii][0] is: ',time_sec_list_list[ii][0])
    # print('time_sec_list_list[ii][-1] is:  ', time_sec_list_list[ii][-1])
    # print('time_smooth_pos is: ', time_smooth_pos[ii])
    # print('time_sec_list_list[ii] is: ', time_sec_list_list[ii])

# time_smooth = np.linspace(time_sec_list_list[ii][0], 
#                     time_sec_list_list[ii][-1], N_time_steps_pos_list[ii])

for ii in range(N_cf):
    pos_x_list_i = np.array(x_list_list[ii])
    pos_y_list_i = np.array(y_list_list[ii])
    pos_z_list_i = np.array(z_list_list[ii])


    time_smooth[ii] = [x - time_smooth[ii][0] for x in time_smooth[ii]]

for ii in range(N_cf):
    ax1.plot(time_smooth[ii], x_list_list[ii])
    legend_pos.append('x_'+str(ii+1))
    ax1.plot(time_smooth[ii], y_list_list[ii])
    legend_pos.append('y_'+str(ii+1))
    ax1.plot(time_smooth[ii], z_list_list[ii])
    legend_pos.append('z_'+str(ii+1))

# print('time_smooth_pos is: ', time_smooth_pos)


ax1.legend(legend_pos)

ax1.set_xlabel('t [s]')
ax1.set_ylabel('x, y, z [m]')


# +++++++++++++++++++++++++ Plotting trajectories 2D ++++++++++++++++++++++++++++++

fig2,ax2 = plt.subplots()
legend_traj = []
plt.title('2D Trajectory')

for ii in range(N_cf):
    ax2.plot(x_list_list[ii], y_list_list[ii])
    ax2.plot(x_list_list[ii][0], y_list_list[ii][0],'bo')
    ax2.plot(x_list_list[ii][-1], y_list_list[ii][-1],'ro')
    legend_traj.append('drone_'+str(ii+1))

ax2.set_xlabel('x [m]')
ax2.set_ylabel('y [m]')

ax2.set_aspect("equal")
plt.grid("minor")


# +++++++++++++++++++++++++ Plotting trajectories 3D ++++++++++++++++++++++++++++++

fig3 = plt.subplots()
legend_traj = []
ax3 = plt.axes(projection="3d")
 
plt.title('3D Trajectory')


for ii in range(N_cf):

    ax3.plot3D(x_list_list[ii], y_list_list[ii], z_list_list[ii])
    legend_traj.append('drone_'+str(ii+1))
    ax3.scatter(x_list_list[ii][0], y_list_list[ii][0], z_list_list[ii][0],'bo')
    ax3.scatter(x_list_list[ii][-1], y_list_list[ii][-1], z_list_list[ii][-1], 'ro')


ax3.set_xlabel('x [m]')
ax3.set_ylabel('y [m]')
ax3.set_zlabel('z [m]')




# +++++++++++++++++++++ Plotting Velocities vs. time +++++++++++++++++++++++++++


fig4,ax4 = plt.subplots()
legend_vel = []
plt.title('Velocities vs. Time')

N_time_steps_vel_list = []

for ii in range(N_cf):
    N_time_steps_vel_list.append(len(vx_list_list[ii]))

time_smooth_vel = []

for ii in range(N_cf):
    time_smooth_vel.append(np.linspace(time_sec_list_list[ii][0], 
                        time_sec_list_list[ii][-1], N_time_steps_vel_list[ii]))

for ii in range(N_cf):
    vel_x_list_i = np.array(vx_list_list[ii])
    vel_y_list_i = np.array(vy_list_list[ii])
    vel_z_list_i = np.array(vz_list_list[ii])


for ii in range(N_cf):
    ax4.plot(time_smooth[ii], vx_list_list[ii])
    legend_vel.append('v_x_'+str(ii+1))
    ax4.plot(time_smooth[ii], vy_list_list[ii])
    legend_vel.append('v_y_'+str(ii+1))
    ax4.plot(time_smooth[ii], vz_list_list[ii])
    legend_vel.append('v_z_'+str(ii+1))

ax4.legend(legend_vel)

ax4.set_xlabel('t [s]')

ax4.set_ylabel('v_x, v_y, v_z [m/s]')



# ++++++++++++++++++ Plotting Absolute Velocity vs. time +++++++++++++++++++++++++


fig5,ax5 = plt.subplots()
legend_vel = []
plt.title('Absolute velocity vs. Time')

for ii in range(N_cf):
    vel_x_list_i = np.array(vx_list_list[ii])
    vel_y_list_i = np.array(vy_list_list[ii])
    vel_z_list_i = np.array(vz_list_list[ii])

    abs_vel_list_i = np.sqrt(np.add(
                np.add(np.square(vel_x_list_i), np.square(vel_y_list_i)),
                np.square(vel_z_list_i)))


    abs_vel_list_list.append(abs_vel_list_i)

for ii in range(N_cf):
    ax5.plot(time_smooth[ii], abs_vel_list_list[ii])
    legend_vel.append('v_'+str(ii+1))


ax5.legend(legend_vel)

ax5.set_xlabel('t [s]')

ax5.set_ylabel('v_abs [m/s]')

# ++++++++++++++++++ Plotting Roll-Pitch-Yaw vs. time ++++++++++++++++++++++++++++


fig6,ax6 = plt.subplots()
legend_attitude = []
plt.title('Attitude vs. Time')

N_time_steps_att_list = []

for ii in range(N_cf):
    N_time_steps_att_list.append(len(roll_list_list[ii]))


time_smooth_att = []

for ii in range(N_cf):
    time_smooth_att.append(np.linspace(time_sec_list_list[ii][0], 
                        time_sec_list_list[ii][-1], N_time_steps_vel_list[ii]))



for ii in range(N_cf):
    ax6.plot(time_smooth[ii], roll_list_list[ii])
    legend_attitude.append('roll_'+ str(ii+1))
    ax6.plot(time_smooth[ii], pitch_list_list[ii])
    legend_attitude.append('pitch_'+ str(ii+1))
    ax6.plot(time_smooth[ii], yaw_list_list[ii])
    legend_attitude.append('yaw_'+ str(ii+1))

ax6.legend(legend_attitude)

ax6.set_xlabel('t [s]')

ax6.set_ylabel('Roll, Pitch, Yaw [deg]')


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


plt.show(block=True)


bag.close()