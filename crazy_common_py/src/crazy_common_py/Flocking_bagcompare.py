import math

import rosbag
import rospkg
from crazy_common_py.common_functions import rad2deg
from crazy_common_py.default_topics import DEFAULT_CF_STATE_TOPIC, DEFAULT_MOTOR_CMD_TOPIC, \
    DEFAULT_ACTUAL_DESTINATION_TOPIC
from matplotlib.pyplot import plot, xlabel, ylabel,show, figure, title, ylim, subplot, xlim, legend, axes, Circle, \
    gca, axis, subplots, scatter
from numpy import array, argmax, linspace, ones, zeros, flip, concatenate, linalg, hstack, mean, absolute, argmin
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

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from itertools import product, combinations

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)


def displayBox(ax, dimensions, center, RPY, color, m_scale=20, lineWidth=1):
    # Non rotated coordinates:
    p_A = Vector3(center[0] - dimensions[0] / 2, center[1] - dimensions[1] / 2, center[2] + dimensions[2] / 2)
    p_B = Vector3(center[0] - dimensions[0] / 2, center[1] + dimensions[1] / 2, center[2] + dimensions[2] / 2)
    p_C = Vector3(center[0] + dimensions[0] / 2, center[1] + dimensions[1] / 2, center[2] + dimensions[2] / 2)
    p_D = Vector3(center[0] + dimensions[0] / 2, center[1] - dimensions[1] / 2, center[2] + dimensions[2] / 2)

    p_E = Vector3(center[0] - dimensions[0] / 2, center[1] - dimensions[1] / 2, center[2] - dimensions[2] / 2)
    p_F = Vector3(center[0] - dimensions[0] / 2, center[1] + dimensions[1] / 2, center[2] - dimensions[2] / 2)
    p_G = Vector3(center[0] + dimensions[0] / 2, center[1] + dimensions[1] / 2, center[2] - dimensions[2] / 2)
    p_H = Vector3(center[0] + dimensions[0] / 2, center[1] - dimensions[1] / 2, center[2] - dimensions[2] / 2)

    p_center = Vector3(center[0], center[1], center[2])
    # Rotate al the cevtors:
    p_A_rot = RotateVector(p_A, Vector3(RPY[0], RPY[1], RPY[2]))
    p_B_rot = RotateVector(p_B, Vector3(RPY[0], RPY[1], RPY[2]))
    p_C_rot = RotateVector(p_C, Vector3(RPY[0], RPY[1], RPY[2]))
    p_D_rot = RotateVector(p_D, Vector3(RPY[0], RPY[1], RPY[2]))

    p_E_rot = RotateVector(p_E, Vector3(RPY[0], RPY[1], RPY[2]))
    p_F_rot = RotateVector(p_F, Vector3(RPY[0], RPY[1], RPY[2]))
    p_G_rot = RotateVector(p_G, Vector3(RPY[0], RPY[1], RPY[2]))
    p_H_rot = RotateVector(p_H, Vector3(RPY[0], RPY[1], RPY[2]))

    p_center_rot = RotateVector(p_center, Vector3(RPY[0], RPY[1], RPY[2]))

    upperFace = [p_A_rot, p_B_rot, p_C_rot, p_D_rot, p_A_rot]
    lowerFace = [p_E_rot, p_F_rot, p_G_rot, p_H_rot, p_E_rot]

    # Translation:
    if RPY[0] != 0 or RPY[1] != 0 or RPY[2] != 0:
        deltax = p_center_rot.x - p_center.x
        deltay = p_center_rot.y - p_center.y
        deltaz = p_center_rot.z - p_center.z
        for ii in range(0, len(upperFace) - 1):
            upperFace[ii].x = upperFace[ii].x - deltax
            lowerFace[ii].x = lowerFace[ii].x - deltax

            upperFace[ii].y = upperFace[ii].y - deltay
            lowerFace[ii].y = lowerFace[ii].y - deltay

            upperFace[ii].z = upperFace[ii].z - deltaz
            lowerFace[ii].z = lowerFace[ii].z - deltaz

    for ii in range(0, 4):
        a = Arrow3D([upperFace[ii].x, upperFace[ii+1].x], [upperFace[ii].y, upperFace[ii+1].y],
                    [upperFace[ii].z, upperFace[ii+1].z], mutation_scale=m_scale,
                    lw=lineWidth, arrowstyle="-", color=color)
        ax.add_artist(a)
        b = Arrow3D([lowerFace[ii].x, lowerFace[ii+1].x], [lowerFace[ii].y, lowerFace[ii+1].y],
                    [lowerFace[ii].z, lowerFace[ii+1].z], mutation_scale=m_scale,
                    lw=lineWidth, arrowstyle="-", color=color)
        ax.add_artist(b)
        c = Arrow3D([upperFace[ii].x, lowerFace[ii].x], [upperFace[ii].y, lowerFace[ii].y],
                    [upperFace[ii].z, lowerFace[ii].z], mutation_scale=m_scale,
                    lw=lineWidth, arrowstyle="-", color=color)
        ax.add_artist(c)

def WireframeSphere(ax, centre, radius, color, alpha=0.5,
                    n_meridians=20, n_circles_latitude=None):
    """
    Create the arrays of values to plot the wireframe of a sphere.

    Parameters
    ----------
    centre: array like
        A point, defined as an iterable of three numerical values.
    radius: number
        The radius of the sphere.
    n_meridians: int
        The number of meridians to display (circles that pass on both poles).
    n_circles_latitude: int
        The number of horizontal circles (akin to the Equator) to display.
        Notice this includes one for each pole, and defaults to 4 or half
        of the *n_meridians* if the latter is larger.

    Returns
    -------
    sphere_x, sphere_y, sphere_z: arrays
        The arrays with the coordinates of the points to make the wireframe.
        Their shape is (n_meridians, n_circles_latitude).

    Examples
    --------
    >>> fig = plt.figure()
    >>> ax = fig.gca(projection='3d')
    >>> ax.set_aspect("equal")
    >>> sphere = ax.plot_wireframe(*WireframeSphere(), color="r", alpha=0.5)
    >>> fig.show()

    >>> fig = plt.figure()
    >>> ax = fig.gca(projection='3d')
    >>> ax.set_aspect("equal")
    >>> frame_xs, frame_ys, frame_zs = WireframeSphere()
    >>> sphere = ax.plot_wireframe(frame_xs, frame_ys, frame_zs, color="r", alpha=0.5)
    >>> fig.show()
    """
    if n_circles_latitude is None:
        n_circles_latitude = max(n_meridians / 2, 4)
    u, v = np.mgrid[0:2 * np.pi:n_meridians * 1j, 0:np.pi:n_circles_latitude * 1j]
    sphere_x = centre[0] + radius * np.cos(u) * np.sin(v)
    sphere_y = centre[1] + radius * np.sin(u) * np.sin(v)
    sphere_z = centre[2] + radius * np.cos(v)
    ax.plot_wireframe(sphere_x, sphere_y, sphere_z, color=color, alpha=alpha)
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
light_black_color = '#34495E'

first_row_red = [orange_color, '#78281F', '#B03A2E', '#E74C3C']
second_row_purple = ['#4A235A', '#6C3483', '#8E44AD', '#BB8FCE']
third_row_blu = ['#1B4F72', '#2874A6', '#3498DB', '#85C1E9']
fourth_row_green = ['#186A3B', '#239B56', '#2ECC71', '#82E0AA']
grid_colors = first_row_red + second_row_purple + third_row_blu + fourth_row_green

# Time precision:
time_round_precision = 1

# Limit radius:
cf_radius = 0.1

# Cf dimension:
cf_dimx = 0.09
cf_dimy = 0.09
cf_dimz = 0.03

# Bags:
bag_states_name = 'sim_states_flocking_N1.bag'
bag_clock_name = 'sim_clock_flocking_N1.bag'
rospack = rospkg.RosPack()
package_path = rospack.get_path('crazyCmd')
bag_states_path = package_path + '/data/output/Rosbags/' + bag_states_name
bag_clock_path = package_path + '/data/output/Rosbags/' + bag_clock_name
bag_states = rosbag.Bag(bag_states_path)
bag_clock = rosbag.Bag(bag_clock_path)

# Functions:
checkCollision = False
safe_radius = 0.2
safe_distance = safe_radius * 2
experiment_N = 1
# Images:
show_initial_pos = False


show_positions_comparison = False
show_velocities_comparison = False
show_collisions = True
show_collisions_3 = False

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
if experiment_N == 3:
    initial_altitude = 2.0

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
    circle_altitude = 1.5
    if experiment_N == 3:
        circle_altitude = 2.5
    ax.plot3D(cf1_ref_x, cf1_ref_circle_y1, ones((len(cf1_ref_x),)) * circle_altitude, color=orange_color)
    ax.plot3D(cf1_ref_x, cf1_ref_circle_y2, ones((len(cf1_ref_x),)) * circle_altitude, color=orange_color)
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
                        print(f'ALERT COLLISION! between Cf{ii+1} and Cf{jj+1}, at time {clock_gazebo[time_instant]}: distance = {distance}')
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
    title('First row: position')
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
    title('First row: velocity')
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
    # 2-6:
    fig_cont += 1
    figure(fig_cont)
    subplot(311)
    final_pos = argmax(clock_gazebo>5)
    plot(clock_gazebo[0:final_pos], positions_x[0:final_pos, 1], color=grid_colors[1], label='Cf2')
    plot(clock_gazebo[0:final_pos], positions_x[0:final_pos, 5], color=grid_colors[5], label='Cf6')
    ylabel('X [m]')
    title('Collision alert 2-6')
    legend(loc='upper right')

    subplot(312)
    plot(clock_gazebo[0:final_pos], positions_y[0:final_pos, 1], color=grid_colors[1], label='Cf2')
    plot(clock_gazebo[0:final_pos], positions_y[0:final_pos, 5], color=grid_colors[5], label='Cf6')
    ylabel('Y [m]')
    legend(loc='upper right')

    subplot(313)
    plot(clock_gazebo[0:final_pos], positions_z[0:final_pos, 1], color=grid_colors[1], label='Cf2')
    plot(clock_gazebo[0:final_pos], positions_z[0:final_pos, 5], color=grid_colors[5], label='Cf6')
    ylabel('Z [m]')
    legend(loc='upper right')
    xlabel('Time [s]')

    # 12-8:
    fig_cont += 1
    figure(fig_cont)
    subplot(311)
    final_pos = argmax(clock_gazebo > 5)
    plot(clock_gazebo[0:final_pos], positions_x[0:final_pos, 11], color=grid_colors[11], label='Cf12')
    plot(clock_gazebo[0:final_pos], positions_x[0:final_pos, 7], color=grid_colors[7], label='Cf8')
    ylabel('X [m]')
    title('Collision alert 12-8')
    legend(loc='upper right')

    subplot(312)
    plot(clock_gazebo[0:final_pos], positions_y[0:final_pos, 11], color=grid_colors[11], label='Cf12')
    plot(clock_gazebo[0:final_pos], positions_y[0:final_pos, 7], color=grid_colors[7], label='Cf8')
    ylabel('Y [m]')
    legend(loc='upper right')

    subplot(313)
    plot(clock_gazebo[0:final_pos], positions_z[0:final_pos, 11], color=grid_colors[11], label='Cf12')
    plot(clock_gazebo[0:final_pos], positions_z[0:final_pos, 7], color=grid_colors[7], label='Cf8')
    ylabel('Z [m]')
    legend(loc='upper right')
    xlabel('Time [s]')

    # 12-16:
    fig_cont += 1
    figure(fig_cont)
    subplot(311)
    final_pos = argmax(clock_gazebo > 5)
    plot(clock_gazebo[0:final_pos], positions_x[0:final_pos, 11], color=grid_colors[11], label='Cf12')
    plot(clock_gazebo[0:final_pos], positions_x[0:final_pos, 15], color=grid_colors[15], label='Cf16')
    ylabel('X [m]')
    title('Collision alert 12-16')
    legend(loc='upper right')

    subplot(312)
    plot(clock_gazebo[0:final_pos], positions_y[0:final_pos, 11], color=grid_colors[11], label='Cf12')
    plot(clock_gazebo[0:final_pos], positions_y[0:final_pos, 15], color=grid_colors[15], label='Cf16')
    ylabel('Y [m]')
    legend(loc='upper right')

    subplot(313)
    plot(clock_gazebo[0:final_pos], positions_z[0:final_pos, 11], color=grid_colors[11], label='Cf12')
    plot(clock_gazebo[0:final_pos], positions_z[0:final_pos, 15], color=grid_colors[15], label='Cf16')
    ylabel('Z [m]')
    legend(loc='upper right')
    xlabel('Time [s]')

    #Difference
    color1 = '#82E0AA'
    color2 = '#F7DC6F'
    color3 = '#E59866'
    fig_cont += 1
    figure(fig_cont)
    subplot(311)
    diff_2_6 = zeros((len(clock_gazebo[0:final_pos]), 1))
    diff_12_8 = zeros((len(clock_gazebo[0:final_pos]), 1))
    diff_12_16 = zeros((len(clock_gazebo[0:final_pos]), 1))
    for ii in range(0, len(clock_gazebo[0:final_pos])):
        diff_2_6[ii, 0] = math.sqrt((positions_x[ii, 1] - positions_x[ii, 5]) ** 2 +
                                    (positions_y[ii, 1] - positions_y[ii, 5]) ** 2 +
                                    (positions_z[ii, 1] - positions_z[ii, 5]) ** 2)
        diff_12_8[ii, 0] = math.sqrt((positions_x[ii, 11] - positions_x[ii, 7]) ** 2 +
                                     (positions_y[ii, 11] - positions_y[ii, 7]) ** 2 +
                                     (positions_z[ii, 11] - positions_z[ii, 7]) ** 2)
        diff_12_16[ii, 0] = math.sqrt((positions_x[ii, 11] - positions_x[ii, 15]) ** 2 +
                                      (positions_y[ii, 11] - positions_y[ii, 15]) ** 2 +
                                      (positions_z[ii, 11] - positions_z[ii, 15]) ** 2)

    min_2_6_pos = argmin(diff_2_6, axis=0)
    min_2_6_pos = min_2_6_pos[0]
    min_12_8_pos = argmin(diff_12_8, axis=0)
    min_12_8_pos = min_12_8_pos[0]
    min_12_16_pos = argmin(diff_12_16, axis=0)
    min_12_16_pos = min_12_16_pos[0]



    plot(clock_gazebo[0:final_pos], ones((len(clock_gazebo[0:final_pos],))) * cf_radius * 2, color=red_color,
         label='Collision limit')
    plot(clock_gazebo[0:final_pos], ones((len(clock_gazebo[0:final_pos], ))) * safe_radius * 2, color=light_black_color,
         label='Safe limit')
    plot(clock_gazebo[0:final_pos], diff_2_6, color=color1, label='Cf2-Cf6')
    ylabel('|xij| [m]')
    legend(loc='upper right')
    title('Relative distance')

    subplot(312)
    plot(clock_gazebo[0:final_pos], ones((len(clock_gazebo[0:final_pos], ))) * cf_radius * 2, color=red_color,
         label='Collision limit')
    plot(clock_gazebo[0:final_pos], ones((len(clock_gazebo[0:final_pos], ))) * safe_radius * 2, color=light_black_color,
         label='Safe limit')
    plot(clock_gazebo[0:final_pos], diff_12_8, color=color2, label='Cf12-Cf8')
    ylabel('|xij| [m]')
    legend(loc='upper right')

    subplot(313)
    plot(clock_gazebo[0:final_pos], ones((len(clock_gazebo[0:final_pos], ))) * cf_radius * 2, color=red_color,
         label='Collision limit')
    plot(clock_gazebo[0:final_pos], ones((len(clock_gazebo[0:final_pos], ))) * safe_radius * 2, color=light_black_color,
         label='Safe limit')
    plot(clock_gazebo[0:final_pos], diff_12_16, color=color3, label='Cf12-Cf16')
    ylabel('|xij| [m]')
    legend(loc='upper right')
    xlabel('Time [s]')

    # 3D trajectories:
    fig_cont += 1
    figure(fig_cont)
    ax = axes(projection='3d')

    for ii in range(1, number_of_cfs):
        ax.scatter3D(initial_positions[ii][0], initial_positions[ii][1], initial_positions[ii][2], color=blu_color)
    # Reference trajectory:
    ax.plot3D(positions_x[0:final_pos, 0], positions_y[0:final_pos, 0], positions_z[0:final_pos, 0],
              color=grid_colors[0])
    ax.scatter3D(positions_x[final_pos, 0], positions_y[final_pos, 0], positions_z[final_pos, 0], color=grid_colors[0])
    ax.plot3D(positions_x[0:final_pos, 1], positions_y[0:final_pos, 1], positions_z[0:final_pos, 1],
              color=grid_colors[1])
    ax.scatter3D(positions_x[final_pos, 1], positions_y[final_pos, 1], positions_z[final_pos, 1], color=grid_colors[1])
    ax.plot3D(positions_x[0:final_pos, 5], positions_y[0:final_pos, 5], positions_z[0:final_pos, 5],
              color=grid_colors[5])
    ax.scatter3D(positions_x[final_pos, 5], positions_y[final_pos, 5], positions_z[final_pos, 5], color=grid_colors[5])
    ax.plot3D(positions_x[0:final_pos, 7], positions_y[0:final_pos, 7], positions_z[0:final_pos, 7],
              color=grid_colors[7])
    ax.scatter3D(positions_x[final_pos, 7], positions_y[final_pos, 7], positions_z[final_pos, 7], color=grid_colors[7])
    ax.plot3D(positions_x[0:final_pos, 11], positions_y[0:final_pos, 11], positions_z[0:final_pos, 11],
              color=grid_colors[11])
    ax.scatter3D(positions_x[final_pos, 11], positions_y[final_pos, 11], positions_z[final_pos, 1], color=grid_colors[11])
    ax.plot3D(positions_x[0:final_pos, 15], positions_y[0:final_pos, 15], positions_z[0:final_pos, 15],
              color=grid_colors[15])
    ax.scatter3D(positions_x[final_pos, 15], positions_y[final_pos, 15], positions_z[final_pos, 15], color=grid_colors[15])
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title('3D collisions')
    legend_elements = [Line2D([0], [0], marker='o', color='w', label='Leader',
                              markerfacecolor=grid_colors[0], markersize=10),
                       Line2D([0], [0], marker='o', color='w', label='Cf2',
                              markerfacecolor=grid_colors[1], markersize=10),
                       Line2D([0], [0], marker='o', color='w', label='Cf6',
                              markerfacecolor=grid_colors[5], markersize=10),
                       Line2D([0], [0], marker='o', color='w', label='Cf8',
                              markerfacecolor=grid_colors[7], markersize=10),
                       Line2D([0], [0], marker='o', color='w', label='Cf12',
                              markerfacecolor=grid_colors[11], markersize=10),
                       Line2D([0], [0], marker='o', color='w', label='Cf16',
                              markerfacecolor=grid_colors[15], markersize=10)
                       ]
    ax.legend(handles=legend_elements, loc='upper right')



    # 3D collision box 2_6
    fig_cont += 1
    figure(fig_cont)
    ax = axes(projection='3d')
    # Initial conditions:
    ax.scatter3D(initial_positions[1][0], initial_positions[1][1], initial_positions[1][2], color=blu_color)
    ax.scatter3D(initial_positions[5][0], initial_positions[5][1], initial_positions[5][2], color=blu_color)
    # Reference trajectory:
    '''ax.plot3D(positions_x[0:min_2_6_pos+1, 0], positions_y[0:min_2_6_pos+1, 0], positions_z[0:min_2_6_pos+1, 0],
              color=grid_colors[0])
    ax.scatter3D(positions_x[min_2_6_pos, 0], positions_y[min_2_6_pos, 0], positions_z[min_2_6_pos, 0],
                 color=grid_colors[0])'''
    # Trajectory Cf2:
    ax.plot3D(positions_x[0:min_2_6_pos+1, 1], positions_y[0:min_2_6_pos+1, 1], positions_z[0:min_2_6_pos+1, 1],
              color=grid_colors[1])
    ax.scatter3D(positions_x[min_2_6_pos, 1], positions_y[min_2_6_pos, 1], positions_z[min_2_6_pos, 1],
                 color=grid_colors[1])
    # Trajectory Cf6:
    ax.plot3D(positions_x[0:min_2_6_pos+1, 5], positions_y[0:min_2_6_pos+1, 5], positions_z[0:min_2_6_pos+1, 5],
              color=grid_colors[5])
    ax.scatter3D(positions_x[min_2_6_pos, 5], positions_y[min_2_6_pos, 5], positions_z[min_2_6_pos, 5],
                 color=grid_colors[5])
    # Box and sphere Cf2:
    displayBox(ax, [cf_dimx, cf_dimy, cf_dimz],
               [positions_x[min_2_6_pos, 1], positions_y[min_2_6_pos, 1], positions_z[min_2_6_pos, 1]],
               [orientations_roll[min_2_6_pos, 1], orientations_pitch[min_2_6_pos, 1],
                orientations_yaw[min_2_6_pos, 1]], color=light_black_color)
    WireframeSphere(ax, [positions_x[min_2_6_pos, 1], positions_y[min_2_6_pos, 1], positions_z[min_2_6_pos, 1]],
                    radius=cf_radius, color=red_color)

    # Box and sphere Cf6:
    displayBox(ax, [cf_dimx, cf_dimy, cf_dimz],
               [positions_x[min_2_6_pos, 5], positions_y[min_2_6_pos, 5], positions_z[min_2_6_pos, 5]],
               [orientations_roll[min_2_6_pos, 5], orientations_pitch[min_2_6_pos, 5],
                orientations_yaw[min_2_6_pos, 5]], color=light_black_color)
    WireframeSphere(ax, [positions_x[min_2_6_pos, 5], positions_y[min_2_6_pos, 5], positions_z[min_2_6_pos, 5]],
                    radius=cf_radius, color=red_color)

    legend_elements = [Line2D([0], [0], marker='o', color='w', label='Cf2',
                              markerfacecolor=grid_colors[2], markersize=10),
                       Line2D([0], [0], marker='o', color='w', label='Cf6',
                              markerfacecolor=grid_colors[5], markersize=10),
                       Line2D([0], [0], marker='o', color=red_color, label='Collision limit',
                              markerfacecolor=red_color, markersize=1)]
    ax.legend(handles=legend_elements, loc='upper right')

    dim_x = 0.5
    dim_y = 0.5
    dim_z = 0.5
    ax.set_xlim3d([positions_x[min_2_6_pos, 1] - dim_x, positions_x[min_2_6_pos, 1] + dim_x])
    ax.set_ylim3d([positions_y[min_2_6_pos, 1] - dim_y, positions_y[min_2_6_pos, 1] + dim_y])
    ax.set_zlim3d([positions_z[min_2_6_pos, 1] - dim_z, positions_z[min_2_6_pos, 1] + dim_z])
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')

    # 3D collision box 8_12
    fig_cont += 1
    figure(fig_cont)
    ax = axes(projection='3d')
    # Initial conditions:
    ax.scatter3D(initial_positions[7][0], initial_positions[7][1], initial_positions[7][2], color=blu_color)
    ax.scatter3D(initial_positions[11][0], initial_positions[11][1], initial_positions[11][2], color=blu_color)
    # Reference trajectory:
    '''ax.plot3D(positions_x[0:min_2_6_pos + 1, 0], positions_y[0:min_2_6_pos + 1, 0], positions_z[0:min_2_6_pos + 1, 0],
              color=grid_colors[0])
    ax.scatter3D(positions_x[min_2_6_pos, 0], positions_y[min_2_6_pos, 0], positions_z[min_2_6_pos, 0],
                 color=grid_colors[0])'''
    # Trajectory Cf8:
    ax.plot3D(positions_x[0:min_12_8_pos + 1, 7], positions_y[0:min_12_8_pos + 1, 7], positions_z[0:min_12_8_pos + 1, 7],
              color=grid_colors[7])
    ax.scatter3D(positions_x[min_12_8_pos, 7], positions_y[min_12_8_pos, 7], positions_z[min_12_8_pos, 7],
                 color=grid_colors[7])
    # Trajectory Cf12:
    ax.plot3D(positions_x[0:min_12_8_pos + 1, 11], positions_y[0:min_12_8_pos + 1, 11], positions_z[0:min_12_8_pos + 1, 11],
              color=grid_colors[11])
    ax.scatter3D(positions_x[min_12_8_pos, 11], positions_y[min_12_8_pos, 11], positions_z[min_12_8_pos, 11],
                 color=grid_colors[11])
    # Box and sphere Cf8:
    displayBox(ax, [cf_dimx, cf_dimy, cf_dimz],
               [positions_x[min_12_8_pos, 7], positions_y[min_12_8_pos, 7], positions_z[min_12_8_pos, 7]],
               [orientations_roll[min_12_8_pos, 7], orientations_pitch[min_12_8_pos, 7],
                orientations_yaw[min_12_8_pos, 7]], color=light_black_color)
    WireframeSphere(ax, [positions_x[min_12_8_pos, 7], positions_y[min_12_8_pos, 7], positions_z[min_12_8_pos, 7]],
                    radius=cf_radius, color=red_color)

    # Box and sphere Cf12:
    displayBox(ax, [cf_dimx, cf_dimy, cf_dimz],
               [positions_x[min_12_8_pos, 11], positions_y[min_12_8_pos, 11], positions_z[min_12_8_pos, 11]],
               [orientations_roll[min_12_8_pos, 11], orientations_pitch[min_12_8_pos, 11],
                orientations_yaw[min_12_8_pos, 11]], color=light_black_color)
    WireframeSphere(ax, [positions_x[min_12_8_pos, 11], positions_y[min_12_8_pos, 11], positions_z[min_12_8_pos, 11]],
                    radius=cf_radius, color=red_color)

    legend_elements = [Line2D([0], [0], marker='o', color='w', label='Cf8',
                              markerfacecolor=grid_colors[7], markersize=10),
                       Line2D([0], [0], marker='o', color='w', label='Cf12',
                              markerfacecolor=grid_colors[11], markersize=10),
                       Line2D([0], [0], marker='o', color=red_color, label='Collision limit',
                              markerfacecolor=red_color, markersize=1)]
    ax.legend(handles=legend_elements, loc='upper right')

    dim_x = 0.5
    dim_y = 0.5
    dim_z = 0.5
    ax.set_xlim3d([positions_x[min_12_8_pos, 7] - dim_x, positions_x[min_12_8_pos, 7] + dim_x])
    ax.set_ylim3d([positions_y[min_12_8_pos, 7] - dim_y, positions_y[min_12_8_pos, 7] + dim_y])
    ax.set_zlim3d([positions_z[min_12_8_pos, 7] - dim_z, positions_z[min_12_8_pos, 7] + dim_z])
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')

    # 3D collision box 12_16
    fig_cont += 1
    figure(fig_cont)
    ax = axes(projection='3d')
    # Initial conditions:
    ax.scatter3D(initial_positions[11][0], initial_positions[11][1], initial_positions[11][2], color=blu_color)
    ax.scatter3D(initial_positions[15][0], initial_positions[15][1], initial_positions[15][2], color=blu_color)
    # Reference trajectory:
    '''ax.plot3D(positions_x[0:min_2_6_pos + 1, 0], positions_y[0:min_2_6_pos + 1, 0], positions_z[0:min_2_6_pos + 1, 0],
              color=grid_colors[0])
    ax.scatter3D(positions_x[min_2_6_pos, 0], positions_y[min_2_6_pos, 0], positions_z[min_2_6_pos, 0],
                 color=grid_colors[0])'''
    # Trajectory Cf12:
    ax.plot3D(positions_x[0:min_12_16_pos + 1, 11], positions_y[0:min_12_16_pos + 1, 11], positions_z[0:min_12_16_pos + 1, 11],
              color=grid_colors[11])
    ax.scatter3D(positions_x[min_12_16_pos, 11], positions_y[min_12_16_pos, 11], positions_z[min_12_16_pos, 11],
                 color=grid_colors[11])
    # Trajectory Cf16:
    ax.plot3D(positions_x[0:min_12_16_pos + 1, 15], positions_y[0:min_12_16_pos + 1, 15],
              positions_z[0:min_12_16_pos + 1, 15],
              color=grid_colors[15])
    ax.scatter3D(positions_x[min_12_16_pos, 15], positions_y[min_12_16_pos, 15], positions_z[min_12_16_pos, 15],
                 color=grid_colors[15])
    # Box and sphere Cf12:
    displayBox(ax, [cf_dimx, cf_dimy, cf_dimz],
               [positions_x[min_12_16_pos, 11], positions_y[min_12_16_pos, 11], positions_z[min_12_16_pos, 11]],
               [orientations_roll[min_12_16_pos, 11], orientations_pitch[min_12_16_pos, 11],
                orientations_yaw[min_12_16_pos, 11]], color=light_black_color)
    WireframeSphere(ax, [positions_x[min_12_16_pos, 11], positions_y[min_12_16_pos, 11], positions_z[min_12_16_pos, 11]],
                    radius=cf_radius, color=red_color)

    # Box and sphere Cf16:
    displayBox(ax, [cf_dimx, cf_dimy, cf_dimz],
               [positions_x[min_12_16_pos, 15], positions_y[min_12_16_pos, 15], positions_z[min_12_16_pos, 15]],
               [orientations_roll[min_12_16_pos, 15], orientations_pitch[min_12_16_pos, 15],
                orientations_yaw[min_12_16_pos, 15]], color=light_black_color)
    WireframeSphere(ax, [positions_x[min_12_16_pos, 15], positions_y[min_12_16_pos, 15], positions_z[min_12_16_pos, 15]],
                    radius=cf_radius, color=red_color)

    legend_elements = [Line2D([0], [0], marker='o', color='w', label='Cf12',
                              markerfacecolor=grid_colors[11], markersize=10),
                       Line2D([0], [0], marker='o', color='w', label='Cf16',
                              markerfacecolor=grid_colors[15], markersize=10),
                       Line2D([0], [0], marker='o', color=red_color, label='Collision limit',
                              markerfacecolor=red_color, markersize=1)]
    ax.legend(handles=legend_elements, loc='upper right')

    dim_x = 0.5
    dim_y = 0.5
    dim_z = 0.5
    ax.set_xlim3d([positions_x[min_12_16_pos, 11] - dim_x, positions_x[min_12_16_pos, 11] + dim_x])
    ax.set_ylim3d([positions_y[min_12_16_pos, 11] - dim_y, positions_y[min_12_16_pos, 11] + dim_y])
    ax.set_zlim3d([positions_z[min_12_16_pos, 11] - dim_z, positions_z[min_12_16_pos, 11] + dim_z])
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')


    # Velocity zoom cf12 and cf16
    fig_cont += 1
    figure(fig_cont)
    subplot(311)
    pos_3s = argmax(clock_gazebo>3)
    pos_1s = argmax(clock_gazebo>1)
    plot(clock_gazebo[pos_1s:pos_3s + 1], velocities_x[pos_1s:pos_3s + 1, 11], color=grid_colors[11], label='Cf12')
    plot(clock_gazebo[pos_1s:pos_3s + 1], velocities_x[pos_1s:pos_3s + 1, 15], color=grid_colors[15], label='Cf16')
    ylabel('Vx [m/s]')
    legend(loc='upper right')
    title('Velocity Cf12-Cf16')

    subplot(312)
    plot(clock_gazebo[pos_1s:pos_3s + 1], velocities_y[pos_1s:pos_3s + 1, 11], color=grid_colors[11], label='Cf12')
    plot(clock_gazebo[pos_1s:pos_3s + 1], velocities_y[pos_1s:pos_3s + 1, 15], color=grid_colors[15], label='Cf16')
    ylabel('Vy [m/s]')
    legend(loc='upper right')

    subplot(313)
    plot(clock_gazebo[pos_1s:pos_3s + 1], velocities_z[pos_1s:pos_3s + 1, 11], color=grid_colors[11], label='Cf12')
    plot(clock_gazebo[pos_1s:pos_3s + 1], velocities_z[pos_1s:pos_3s + 1, 15], color=grid_colors[15], label='Cf16')
    ylabel('Vz [m/s]')
    legend(loc='upper right')
    xlabel('Time [s]')

    # Position zoom cf12 and cf16
    fig_cont += 1
    figure(fig_cont)
    subplot(311)
    pos_3s = argmax(clock_gazebo > 3)
    pos_1s = argmax(clock_gazebo > 1)
    plot(clock_gazebo[pos_1s:pos_3s + 1], positions_x[pos_1s:pos_3s + 1, 11], color=grid_colors[11], label='Cf12')
    plot(clock_gazebo[pos_1s:pos_3s + 1], positions_x[pos_1s:pos_3s + 1, 15], color=grid_colors[15], label='Cf16')
    ylabel('X [m]')
    legend(loc='upper right')
    title('Position Cf12-Cf16')

    subplot(312)
    plot(clock_gazebo[pos_1s:pos_3s + 1], positions_y[pos_1s:pos_3s + 1, 11], color=grid_colors[11], label='Cf12')
    plot(clock_gazebo[pos_1s:pos_3s + 1], positions_y[pos_1s:pos_3s + 1, 15], color=grid_colors[15], label='Cf16')
    ylabel('Y [m]')
    legend(loc='upper right')

    subplot(313)
    plot(clock_gazebo[pos_1s:pos_3s + 1], positions_z[pos_1s:pos_3s + 1, 11], color=grid_colors[11], label='Cf12')
    plot(clock_gazebo[pos_1s:pos_3s + 1], positions_z[pos_1s:pos_3s + 1, 15], color=grid_colors[15], label='Cf16')
    ylabel('Z [m]')
    legend(loc='upper right')
    xlabel('Time [s]')



if show_collisions_3:
    # 1-9:
    fig_cont += 1
    figure(fig_cont)
    subplot(311)
    final_pos = argmax(clock_gazebo>5)
    plot(clock_gazebo[0:final_pos], positions_x[0:final_pos, 0], color=grid_colors[0], label='Leader')
    plot(clock_gazebo[0:final_pos], positions_x[0:final_pos, 8], color=grid_colors[8], label='Cf9')
    ylabel('X [m]')
    title('Collision alert Leader-Cf9')
    legend(loc='upper right')

    subplot(312)
    plot(clock_gazebo[0:final_pos], positions_y[0:final_pos, 0], color=grid_colors[0], label='Leader')
    plot(clock_gazebo[0:final_pos], positions_y[0:final_pos, 8], color=grid_colors[8], label='Cf9')
    ylabel('Y [m]')
    legend(loc='upper right')

    subplot(313)
    plot(clock_gazebo[0:final_pos], positions_z[0:final_pos, 0], color=grid_colors[0], label='Leader')
    plot(clock_gazebo[0:final_pos], positions_z[0:final_pos, 8], color=grid_colors[8], label='Cf9')
    ylabel('Z [m]')
    legend(loc='upper right')
    xlabel('Time [s]')

    #Difference
    color1 = '#82E0AA'
    color2 = '#F7DC6F'
    color3 = '#E59866'
    fig_cont += 1
    figure(fig_cont)

    diff_1_9 = zeros((len(clock_gazebo[0:final_pos]), 1))
    for ii in range(0, len(clock_gazebo[0:final_pos])):
        diff_1_9[ii, 0] = math.sqrt((positions_x[ii, 0] - positions_x[ii, 8]) ** 2 +
                                    (positions_y[ii, 0] - positions_y[ii, 8]) ** 2 +
                                    (positions_z[ii, 0] - positions_z[ii, 8]) ** 2)


    min_1_9_pos = argmin(diff_1_9, axis=0)
    min_1_9_pos = min_1_9_pos[0]


    plot(clock_gazebo[0:final_pos], ones((len(clock_gazebo[0:final_pos],))) * cf_radius * 2, color=red_color,
         label='Collision limit')
    plot(clock_gazebo[0:final_pos], ones((len(clock_gazebo[0:final_pos], ))) * safe_radius * 2, color=light_black_color,
         label='Safe limit')
    plot(clock_gazebo[0:final_pos], diff_1_9, color=color1, label='Leader-Cf9')
    ylabel('|xij| [m]')
    legend(loc='upper right')
    title('Relative distance')


    # 3D trajectories:
    fig_cont += 1
    figure(fig_cont)
    ax = axes(projection='3d')

    for ii in range(1, number_of_cfs):
        ax.scatter3D(initial_positions[ii][0], initial_positions[ii][1], initial_positions[ii][2], color=blu_color)
    # Reference trajectory:
    ax.plot3D(positions_x[0:final_pos, 0], positions_y[0:final_pos, 0], positions_z[0:final_pos, 0],
              color=grid_colors[0])
    ax.scatter3D(positions_x[final_pos, 0], positions_y[final_pos, 0], positions_z[final_pos, 0], color=grid_colors[0])
    ax.plot3D(positions_x[0:final_pos, 8], positions_y[0:final_pos, 8], positions_z[0:final_pos, 8],
              color=grid_colors[8])
    ax.scatter3D(positions_x[final_pos, 8], positions_y[final_pos, 8], positions_z[final_pos, 8], color=grid_colors[8])
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title('3D collisions')
    legend_elements = [Line2D([0], [0], marker='o', color='w', label='Leader',
                              markerfacecolor=grid_colors[0], markersize=10),
                       Line2D([0], [0], marker='o', color='w', label='Cf9',
                              markerfacecolor=grid_colors[8], markersize=10)]
    ax.legend(handles=legend_elements, loc='upper right')



    # 3D collision box 1_9
    fig_cont += 1
    figure(fig_cont)
    ax = axes(projection='3d')
    # Initial conditions:
    #ax.scatter3D(initial_positions[0][0], initial_positions[0][1], initial_positions[0][2], color=blu_color)
    ax.scatter3D(initial_positions[8][0], initial_positions[8][1], initial_positions[8][2], color=blu_color)
    # Reference trajectory:
    '''ax.plot3D(positions_x[0:min_2_6_pos+1, 0], positions_y[0:min_2_6_pos+1, 0], positions_z[0:min_2_6_pos+1, 0],
              color=grid_colors[0])
    ax.scatter3D(positions_x[min_2_6_pos, 0], positions_y[min_2_6_pos, 0], positions_z[min_2_6_pos, 0],
                 color=grid_colors[0])'''
    # Trajectory Cf1:
    ax.plot3D(positions_x[0:min_1_9_pos+1, 0], positions_y[0:min_1_9_pos+1, 0], positions_z[0:min_1_9_pos+1, 0],
              color=grid_colors[0])
    ax.scatter3D(positions_x[min_1_9_pos, 0], positions_y[min_1_9_pos, 0], positions_z[min_1_9_pos, 0],
                 color=grid_colors[0])
    # Trajectory Cf9:
    ax.plot3D(positions_x[0:min_1_9_pos+1, 8], positions_y[0:min_1_9_pos+1, 8], positions_z[0:min_1_9_pos+1, 8],
              color=grid_colors[8])
    ax.scatter3D(positions_x[min_1_9_pos, 8], positions_y[min_1_9_pos, 8], positions_z[min_1_9_pos, 8],
                 color=grid_colors[8])
    # Box and sphere Cf1:
    displayBox(ax, [cf_dimx, cf_dimy, cf_dimz],
               [positions_x[min_1_9_pos, 0], positions_y[min_1_9_pos, 0], positions_z[min_1_9_pos, 0]],
               [orientations_roll[min_1_9_pos, 0], orientations_pitch[min_1_9_pos, 0],
                orientations_yaw[min_1_9_pos, 0]], color=light_black_color)
    WireframeSphere(ax, [positions_x[min_1_9_pos, 0], positions_y[min_1_9_pos, 0], positions_z[min_1_9_pos, 0]],
                    radius=cf_radius, color=red_color)

    # Box and sphere Cf9:
    displayBox(ax, [cf_dimx, cf_dimy, cf_dimz],
               [positions_x[min_1_9_pos, 8], positions_y[min_1_9_pos, 8], positions_z[min_1_9_pos, 8]],
               [orientations_roll[min_1_9_pos, 8], orientations_pitch[min_1_9_pos, 8],
                orientations_yaw[min_1_9_pos, 8]], color=light_black_color)
    WireframeSphere(ax, [positions_x[min_1_9_pos, 8], positions_y[min_1_9_pos, 8], positions_z[min_1_9_pos, 8]],
                    radius=cf_radius, color=red_color)

    legend_elements = [Line2D([0], [0], marker='o', color='w', label='Leader',
                              markerfacecolor=grid_colors[0], markersize=10),
                       Line2D([0], [0], marker='o', color='w', label='Cf9',
                              markerfacecolor=grid_colors[8], markersize=10),
                       Line2D([0], [0], marker='o', color=red_color, label='Collision limit',
                              markerfacecolor=red_color, markersize=1)]
    ax.legend(handles=legend_elements, loc='upper right')

    dim_x = 0.5
    dim_y = 0.5
    dim_z = 0.5
    ax.set_xlim3d([positions_x[min_1_9_pos, 0] - dim_x, positions_x[min_1_9_pos, 0] + dim_x])
    ax.set_ylim3d([positions_y[min_1_9_pos, 0] - dim_y, positions_y[min_1_9_pos, 0] + dim_y])
    ax.set_zlim3d([positions_z[min_1_9_pos, 0] - dim_z, positions_z[min_1_9_pos, 0] + dim_z])
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')



    # Velocity zoom cf1 and cf9
    fig_cont += 1
    figure(fig_cont)
    subplot(311)
    pos_3s = argmax(clock_gazebo>3)
    pos_1s = argmax(clock_gazebo>1)
    plot(clock_gazebo[pos_1s:pos_3s + 1], velocities_x[pos_1s:pos_3s + 1, 0], color=grid_colors[0], label='Leader')
    plot(clock_gazebo[pos_1s:pos_3s + 1], velocities_x[pos_1s:pos_3s + 1, 8], color=grid_colors[8], label='Cf9')
    ylabel('Vx [m/s]')
    legend(loc='upper right')
    title('Velocity Leader-Cf9')

    subplot(312)
    plot(clock_gazebo[pos_1s:pos_3s + 1], velocities_y[pos_1s:pos_3s + 1, 0], color=grid_colors[0], label='Leader')
    plot(clock_gazebo[pos_1s:pos_3s + 1], velocities_y[pos_1s:pos_3s + 1, 8], color=grid_colors[8], label='Cf9')
    ylabel('Vy [m/s]')
    legend(loc='upper right')

    subplot(313)
    plot(clock_gazebo[pos_1s:pos_3s + 1], velocities_z[pos_1s:pos_3s + 1, 0], color=grid_colors[0], label='Leader')
    plot(clock_gazebo[pos_1s:pos_3s + 1], velocities_z[pos_1s:pos_3s + 1, 8], color=grid_colors[8], label='Cf9')
    ylabel('Vz [m/s]')
    legend(loc='upper right')
    xlabel('Time [s]')

show()