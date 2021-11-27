import math

import rosbag
import rospkg
from crazy_common_py.common_functions import rad2deg
from crazy_common_py.default_topics import DEFAULT_CF_STATE_TOPIC, DEFAULT_MOTOR_CMD_TOPIC, \
    DEFAULT_ACTUAL_DESTINATION_TOPIC
from matplotlib.pyplot import plot, xlabel, ylabel,show, figure, title, ylim, subplot, xlim, legend, axes, Circle, \
    gca, axis, subplots, scatter
from numpy import array, argmax, linspace, ones, zeros, flip, concatenate, linalg, hstack, mean
from enum import Enum
from crazyflie_messages.msg import SwarmStates
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch, Ellipse
from matplotlib.patches import Patch
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d import proj3d
from crazy_common_py.common_functions import Vector3, RotateVector, deg2rad, rad2deg
from pykalman import KalmanFilter
from scipy.signal import savgol_filter, correlate

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)

# ======================================================================================================================
#                                                   S E T T I N G S
# ======================================================================================================================
# Colors:
blu_color = '#0047AB'
light_blu1_color = '#6495ED'
light_blu2_color = '#0096FF'
center_color = '#FFA500'
center_path_color = '#FF6347'
red_color = '#EC7073'
green_color = '#58D68D'
orange_color = '#F5B041'

first_row_red = [orange_color, '#78281F', '#B03A2E', '#E74C3C']
second_row_purple = ['#4A235A', '#6C3483', '#8E44AD', '#BB8FCE']
third_row_blu = ['#1B4F72', '#2874A6', '#3498DB', '#85C1E9']
fourth_row_green = ['#186A3B', '#239B56', '#2ECC71', '#82E0AA']
grid_colors = first_row_red + second_row_purple + third_row_blu + fourth_row_green

# Time precision:
time_round_precision = 1

# Bags:
bag_states_name = 'sim_states_flocking_N3.bag'
bag_clock_name = 'sim_clock_flocking_N3.bag'
rospack = rospkg.RosPack()
package_path = rospack.get_path('crazyCmd')
bag_states_path = package_path + '/data/output/Rosbags/' + bag_states_name
bag_clock_path = package_path + '/data/output/Rosbags/' + bag_clock_name
bag_states = rosbag.Bag(bag_states_path)
bag_clock = rosbag.Bag(bag_clock_path)

# Functions:
checkCollision = True
safe_radius = 0.2
safe_distance = safe_radius * 2

# Images:
show_initial_pos = False

show_positions_comparison = True
show_velocities_comparison = True
show_collisions = True
# ======================================================================================================================
#                                             D A T A  E X T R A C T I O N
# ======================================================================================================================
# States bag extraction:
states_collection = []
for topic, msg, t in bag_states.read_messages(topics=['/swarm/states']):
    states_collection.append(msg.states)
states_secs = round(bag_states.get_end_time() - bag_states.get_start_time(), time_round_precision)
bag_states.close()

# Time extraction:
clock_gazebo = []
for topic, msg, t in bag_clock.read_messages(topics=['/swarm/states']):
    clock_gazebo.append(msg.time.to_sec())
clock_secs = round(bag_clock.get_end_time() - bag_clock.get_start_time(), time_round_precision)
bag_clock.close()
clock_gazebo = linspace(0, states_secs, len(states_collection))
# ======================================================================================================================
#                                      S T A T E S  D A T A  O R G A N I Z A T I O N
# ======================================================================================================================
number_of_cfs = len(states_collection[0])

positions_x = zeros((len(states_collection), number_of_cfs))
positions_y = zeros((len(states_collection), number_of_cfs))
positions_z = zeros((len(states_collection), number_of_cfs))

velocities_x = zeros((len(states_collection), number_of_cfs))
velocities_y = zeros((len(states_collection), number_of_cfs))
velocities_z = zeros((len(states_collection), number_of_cfs))

orientations_roll = zeros((len(states_collection), number_of_cfs))
orientations_pitch = zeros((len(states_collection), number_of_cfs))
orientations_yaw = zeros((len(states_collection), number_of_cfs))

angular_velocities_x = zeros((len(states_collection), number_of_cfs))
angular_velocities_y = zeros((len(states_collection), number_of_cfs))
angular_velocities_z = zeros((len(states_collection), number_of_cfs))

for time_instant in range(0, len(states_collection)):
    for cf in range(0, number_of_cfs):
        positions_x[time_instant, cf] = states_collection[time_instant][cf].position.x
        positions_y[time_instant, cf] = states_collection[time_instant][cf].position.y
        positions_z[time_instant, cf] = states_collection[time_instant][cf].position.z

        velocities_x[time_instant, cf] = states_collection[time_instant][cf].velocity.x
        velocities_y[time_instant, cf] = states_collection[time_instant][cf].velocity.y
        velocities_z[time_instant, cf] = states_collection[time_instant][cf].velocity.z

        orientations_roll[time_instant, cf] = states_collection[time_instant][cf].orientation.roll
        orientations_pitch[time_instant, cf] = states_collection[time_instant][cf].orientation.pitch
        orientations_yaw[time_instant, cf] = states_collection[time_instant][cf].orientation.yaw

        angular_velocities_x[time_instant, cf] = states_collection[time_instant][cf].rotating_speed.x
        angular_velocities_y[time_instant, cf] = states_collection[time_instant][cf].rotating_speed.y
        angular_velocities_z[time_instant, cf] = states_collection[time_instant][cf].rotating_speed.z

# ======================================================================================================================
#                                               R E F E R E N C E
# ======================================================================================================================
# Initial positions:
initial_altitude = 1.0
initial_positions = [[2, 0, initial_altitude + 0.5],
                     [0, 1, initial_altitude],
                     [0, 2, initial_altitude],
                     [0, 3, initial_altitude],
                     [1, 0, initial_altitude],
                     [1, 1, initial_altitude],
                     [1, 2, initial_altitude],
                     [1, 3, initial_altitude],
                     [2, 0, initial_altitude],
                     [2, 1, initial_altitude],
                     [2, 2, initial_altitude],
                     [2, 3, initial_altitude],
                     [3, 0, initial_altitude],
                     [3, 1, initial_altitude],
                     [3, 2, initial_altitude],
                     [3, 3, initial_altitude]]
# Leader trajectory:
radius = 2.0
cf1_ref_circle_y1 = []
cf1_ref_circle_y2 = []
n_points_ref_circle = 1000
cf1_ref_x = linspace(0.0, 4.0, n_points_ref_circle)
for ii in range(0, n_points_ref_circle):
    cf1_ref_circle_y1.append(math.sqrt(radius ** 2 - (cf1_ref_x[ii] - 2) ** 2) + 2)
    cf1_ref_circle_y2.append(- math.sqrt(radius ** 2 - (cf1_ref_x[ii] - 2) ** 2) + 2)

# ======================================================================================================================
#                                               R E F E R E N C E
# ======================================================================================================================
fig_cont = 0
# ----------------------------------------------------------------------------------------------------------------------
#               I N I T I A L  F O R M A T I O N  A N D  R E F E R E N C E  L E A D E R  T R A J E C T O R Y
# ----------------------------------------------------------------------------------------------------------------------
if show_initial_pos:
    fig_cont += 1
    figure(fig_cont)
    ax = axes(projection='3d')
    # Initial positions:
    ax.scatter3D(initial_positions[0][0], initial_positions[0][1], initial_positions[0][2], color=orange_color)
    for ii in range(1, number_of_cfs):
        ax.scatter3D(initial_positions[ii][0], initial_positions[ii][1], initial_positions[ii][2], color=blu_color)
    # Reference trajectory:
    ax.plot3D(cf1_ref_x, cf1_ref_circle_y1, ones((len(cf1_ref_x),)) * 1.5, color=orange_color)
    ax.plot3D(cf1_ref_x, cf1_ref_circle_y2, ones((len(cf1_ref_x),)) * 1.5, color=orange_color)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title('Initial conditions')
    legend_elements = [Line2D([0], [0], color=orange_color, lw=2, label='Reference'),
                       Line2D([0], [0], marker='o', color='w', label='Leader',
                              markerfacecolor=orange_color, markersize=10),
                       Line2D([0], [0], marker='o', color='w', label='Followers',
                              markerfacecolor=blu_color, markersize=10)]
    ax.legend(handles=legend_elements, loc='upper right')

# ======================================================================================================================
#                                       C O L L I S I O N  D E T E C T I O N
# ======================================================================================================================
if checkCollision:
    cont = 0
    for time_instant in range(0, len(clock_gazebo)):
        for ii in range(0, number_of_cfs):
            for jj in range(0, number_of_cfs):
                if ii != jj:
                    distance = math.sqrt((positions_x[time_instant, ii] - positions_x[time_instant, jj]) ** 2 +
                                         (positions_y[time_instant, ii] - positions_y[time_instant, jj]) ** 2 +
                                         (positions_z[time_instant, ii] - positions_z[time_instant, jj]) ** 2)
                    if distance <= safe_distance:
                        print(f'ALERT COLLISION! between Cf{ii} and Cf{jj}, at time {clock_gazebo[time_instant]}: distance = {distance}')
                        cont += 1
    print(f'TOTAL POSSIBLE COLLISIONS: {cont}')

# ======================================================================================================================
#                                                F I G U R E S
# ======================================================================================================================
# ----------------------------------------------------------------------------------------------------------------------
#                                       P O S I T I O N S  C O M P A R I S O N
# ----------------------------------------------------------------------------------------------------------------------
if show_positions_comparison:
    # FIRST ROW
    fig_cont += 1
    figure(fig_cont)
    subplot(311)
    cont = 2
    for ii in range(1, 4):
        tmp_label = 'Cf' + str(cont)
        plot(clock_gazebo, positions_x[:, ii], color=grid_colors[ii], label=tmp_label)
        cont += 1
    plot(clock_gazebo, positions_x[:, 0], color=grid_colors[0], label='Leader')
    ylabel('X [m]')
    title('Firs row: position')
    legend(loc='upper right')
    subplot(312)
    cont = 2
    for ii in range(1, 4):
        tmp_label = 'Cf' + str(cont)
        plot(clock_gazebo, positions_y[:, ii], color=grid_colors[ii], label=tmp_label)
        cont += 1
    plot(clock_gazebo, positions_y[:, 0], color=grid_colors[0], label='Leader')
    ylabel('Y [m]')
    legend(loc='upper right')
    subplot(313)
    cont = 2
    for ii in range(1, 4):
        tmp_label = 'Cf' + str(cont)
        plot(clock_gazebo, positions_z[:, ii], color=grid_colors[ii], label=tmp_label)
        cont += 1
    plot(clock_gazebo, positions_z[:, 0], color=grid_colors[0], label='Leader')
    ylabel('Z [m]')
    legend(loc='upper right')
    xlabel('Time [s]')

    # SECOND ROW
    fig_cont += 1
    figure(fig_cont)
    subplot(311)
    cont = 5
    for ii in range(4, 8):
        tmp_label = 'Cf' + str(cont)
        plot(clock_gazebo, positions_x[:, ii], color=grid_colors[ii], label=tmp_label)
        cont += 1
    plot(clock_gazebo, positions_x[:, 0], color=grid_colors[0], label='Leader')
    ylabel('X [m]')
    title('Second row: position')
    legend(loc='upper right')
    subplot(312)
    cont = 5
    for ii in range(4, 8):
        tmp_label = 'Cf' + str(cont)
        plot(clock_gazebo, positions_y[:, ii], color=grid_colors[ii], label=tmp_label)
        cont += 1
    plot(clock_gazebo, positions_y[:, 0], color=grid_colors[0], label='Leader')
    ylabel('Y [m]')
    legend(loc='upper right')
    subplot(313)
    cont = 5
    for ii in range(4, 8):
        tmp_label = 'Cf' + str(cont)
        plot(clock_gazebo, positions_z[:, ii], color=grid_colors[ii], label=tmp_label)
        cont += 1
    plot(clock_gazebo, positions_z[:, 0], color=grid_colors[0], label='Leader')
    ylabel('Z [m]')
    legend(loc='upper right')
    xlabel('Time [s]')

    # THIRD ROW
    fig_cont += 1
    figure(fig_cont)
    subplot(311)
    cont = 9
    for ii in range(8, 12):
        tmp_label = 'Cf' + str(cont)
        plot(clock_gazebo, positions_x[:, ii], color=grid_colors[ii], label=tmp_label)
        cont += 1
    plot(clock_gazebo, positions_x[:, 0], color=grid_colors[0], label='Leader')
    ylabel('X [m]')
    title('Third row: position')
    legend(loc='upper right')
    subplot(312)
    cont = 9
    for ii in range(8, 12):
        tmp_label = 'Cf' + str(cont)
        plot(clock_gazebo, positions_y[:, ii], color=grid_colors[ii], label=tmp_label)
        cont += 1
    plot(clock_gazebo, positions_y[:, 0], color=grid_colors[0], label='Leader')
    ylabel('Y [m]')
    legend(loc='upper right')
    subplot(313)
    cont = 9
    for ii in range(8, 12):
        tmp_label = 'Cf' + str(cont)
        plot(clock_gazebo, positions_z[:, ii], color=grid_colors[ii], label=tmp_label)
        cont += 1
    plot(clock_gazebo, positions_z[:, 0], color=grid_colors[0], label='Leader')
    ylabel('Z [m]')
    legend(loc='upper right')
    xlabel('Time [s]')

    # FOURTH ROW
    fig_cont += 1
    figure(fig_cont)
    subplot(311)
    cont = 13
    for ii in range(12, 16):
        tmp_label = 'Cf' + str(cont)
        plot(clock_gazebo, positions_x[:, ii], color=grid_colors[ii], label=tmp_label)
        cont += 1
    plot(clock_gazebo, positions_x[:, 0], color=grid_colors[0], label='Leader')
    ylabel('X [m]')
    title('Fourth row: position')
    legend(loc='upper right')
    subplot(312)
    cont = 13
    for ii in range(12, 16):
        tmp_label = 'Cf' + str(cont)
        plot(clock_gazebo, positions_y[:, ii], color=grid_colors[ii], label=tmp_label)
        cont += 1
    plot(clock_gazebo, positions_y[:, 0], color=grid_colors[0], label='Leader')
    ylabel('Y [m]')
    legend(loc='upper right')
    subplot(313)
    cont = 13
    for ii in range(12, 16):
        tmp_label = 'Cf' + str(cont)
        plot(clock_gazebo, positions_z[:, ii], color=grid_colors[ii], label=tmp_label)
        cont += 1
    plot(clock_gazebo, positions_z[:, 0], color=grid_colors[0], label='Leader')
    ylabel('Z [m]')
    legend(loc='upper right')
    xlabel('Time [s]')

# ----------------------------------------------------------------------------------------------------------------------
#                                       V E L O C I T I E S  C O M P A R I S O N
# ----------------------------------------------------------------------------------------------------------------------
if show_velocities_comparison:
    # FIRST ROW
    fig_cont += 1
    figure(fig_cont)
    subplot(311)
    cont = 2
    for ii in range(1, 4):
        tmp_label = 'Cf' + str(cont)
        plot(clock_gazebo, velocities_x[:, ii], color=grid_colors[ii], label=tmp_label)
        cont += 1
    plot(clock_gazebo, velocities_x[:, 0], color=grid_colors[0], label='Leader')
    ylabel('Vx [m/s]')
    title('Firs row: velocity')
    legend(loc='upper right')
    subplot(312)
    cont = 2
    for ii in range(1, 4):
        tmp_label = 'Cf' + str(cont)
        plot(clock_gazebo, velocities_y[:, ii], color=grid_colors[ii], label=tmp_label)
        cont += 1
    plot(clock_gazebo, velocities_y[:, 0], color=grid_colors[0], label='Leader')
    ylabel('Vy [m/s]')
    legend(loc='upper right')
    subplot(313)
    cont = 2
    for ii in range(1, 4):
        tmp_label = 'Cf' + str(cont)
        plot(clock_gazebo, velocities_z[:, ii], color=grid_colors[ii], label=tmp_label)
        cont += 1
    plot(clock_gazebo, velocities_z[:, 0], color=grid_colors[0], label='Leader')
    ylabel('Vz [m/s]')
    legend(loc='upper right')
    xlabel('Time [s]')

    # SECOND ROW
    fig_cont += 1
    figure(fig_cont)
    subplot(311)
    cont = 5
    for ii in range(4, 8):
        tmp_label = 'Cf' + str(cont)
        plot(clock_gazebo, velocities_x[:, ii], color=grid_colors[ii], label=tmp_label)
        cont += 1
    plot(clock_gazebo, velocities_x[:, 0], color=grid_colors[0], label='Leader')
    ylabel('Vx [m/s]')
    title('Second row: velocity')
    legend(loc='upper right')
    subplot(312)
    cont = 5
    for ii in range(4, 8):
        tmp_label = 'Cf' + str(cont)
        plot(clock_gazebo, velocities_y[:, ii], color=grid_colors[ii], label=tmp_label)
        cont += 1
    plot(clock_gazebo, velocities_y[:, 0], color=grid_colors[0], label='Leader')
    ylabel('Vy [m/s]')
    legend(loc='upper right')
    subplot(313)
    cont = 5
    for ii in range(4, 8):
        tmp_label = 'Cf' + str(cont)
        plot(clock_gazebo, velocities_z[:, ii], color=grid_colors[ii], label=tmp_label)
        cont += 1
    plot(clock_gazebo, velocities_z[:, 0], color=grid_colors[0], label='Leader')
    ylabel('Vz [m/s]')
    legend(loc='upper right')
    xlabel('Time [s]')

    # THIRD ROW
    fig_cont += 1
    figure(fig_cont)
    subplot(311)
    cont = 9
    for ii in range(8, 12):
        tmp_label = 'Cf' + str(cont)
        plot(clock_gazebo, velocities_x[:, ii], color=grid_colors[ii], label=tmp_label)
        cont += 1
    plot(clock_gazebo, velocities_x[:, 0], color=grid_colors[0], label='Leader')
    ylabel('Vx [m/s]')
    title('Third row: velocity')
    legend(loc='upper right')
    subplot(312)
    cont = 9
    for ii in range(8, 12):
        tmp_label = 'Cf' + str(cont)
        plot(clock_gazebo, velocities_y[:, ii], color=grid_colors[ii], label=tmp_label)
        cont += 1
    plot(clock_gazebo, velocities_y[:, 0], color=grid_colors[0], label='Leader')
    ylabel('Vy [m/s]')
    legend(loc='upper right')
    subplot(313)
    cont = 9
    for ii in range(8, 12):
        tmp_label = 'Cf' + str(cont)
        plot(clock_gazebo, velocities_z[:, ii], color=grid_colors[ii], label=tmp_label)
        cont += 1
    plot(clock_gazebo, velocities_z[:, 0], color=grid_colors[0], label='Leader')
    ylabel('Vz [m/s]')
    legend(loc='upper right')
    xlabel('Time [s]')

    # FOURTH ROW
    fig_cont += 1
    figure(fig_cont)
    subplot(311)
    cont = 13
    for ii in range(12, 16):
        tmp_label = 'Cf' + str(cont)
        plot(clock_gazebo, velocities_x[:, ii], color=grid_colors[ii], label=tmp_label)
        cont += 1
    plot(clock_gazebo, velocities_x[:, 0], color=grid_colors[0], label='Leader')
    ylabel('Vx [m/s]')
    title('Fourth row: velocity')
    legend(loc='upper right')
    subplot(312)
    cont = 13
    for ii in range(12, 16):
        tmp_label = 'Cf' + str(cont)
        plot(clock_gazebo, velocities_y[:, ii], color=grid_colors[ii], label=tmp_label)
        cont += 1
    plot(clock_gazebo, velocities_y[:, 0], color=grid_colors[0], label='Leader')
    ylabel('Vy [m/s]')
    legend(loc='upper right')
    subplot(313)
    cont = 13
    for ii in range(12, 16):
        tmp_label = 'Cf' + str(cont)
        plot(clock_gazebo, velocities_z[:, ii], color=grid_colors[ii], label=tmp_label)
        cont += 1
    plot(clock_gazebo, velocities_z[:, 0], color=grid_colors[0], label='Leader')
    ylabel('Vz [m/s]')
    legend(loc='upper right')
    xlabel('Time [s]')

# ----------------------------------------------------------------------------------------------------------------------
#                                       C O L L I S I O N S  D E T E C T I O N S
# ----------------------------------------------------------------------------------------------------------------------
if show_collisions:
    fig_cont += 1
    figure(fig_cont)
    subplot(311)
    final_pos = argmax(clock_gazebo>5)
    plot(clock_gazebo[0:final_pos], positions_x[0:final_pos, 0], color=grid_colors[0], label='Leader')
    plot(clock_gazebo[0:final_pos], positions_x[0:final_pos, 4], color=grid_colors[4], label='Cf5')
    ylabel('X [m]')
    title('Firs row: velocity')
    legend(loc='upper right')

    subplot(312)
    plot(clock_gazebo[0:final_pos], positions_y[0:final_pos, 0], color=grid_colors[0], label='Leader')
    plot(clock_gazebo[0:final_pos], positions_y[0:final_pos, 4], color=grid_colors[4], label='Cf5')
    ylabel('Y [m]')
    legend(loc='upper right')

    subplot(313)
    plot(clock_gazebo[0:final_pos], positions_z[0:final_pos, 0], color=grid_colors[0], label='Leader')
    plot(clock_gazebo[0:final_pos], positions_z[0:final_pos, 4], color=grid_colors[4], label='Cf5')
    ylabel('Z [m]')
    legend(loc='upper right')
    xlabel('Time [s]')

show()