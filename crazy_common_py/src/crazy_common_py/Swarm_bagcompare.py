import math

import rosbag
import rospkg
from crazy_common_py.common_functions import rad2deg
from crazy_common_py.default_topics import DEFAULT_CF_STATE_TOPIC, DEFAULT_MOTOR_CMD_TOPIC, \
    DEFAULT_ACTUAL_DESTINATION_TOPIC
from matplotlib.pyplot import plot, xlabel, ylabel,show, figure, title, ylim, subplot, xlim, legend, axes, Circle, \
    gca, axis, subplots
from numpy import array, argmax, linspace, ones, zeros, flip, concatenate, linalg, hstack, mean
from enum import Enum
from crazyflie_messages.msg import SwarmStates
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch, Ellipse
from mpl_toolkits.mplot3d import proj3d
from crazy_common_py.common_functions import Vector3, RotateVector, deg2rad, rad2deg
from pykalman import KalmanFilter
from scipy.signal import savgol_filter, correlate


class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)

class TaskType(Enum):
    LINEAR_AND_ROUNDS = 0
    ROUNDS = 1
    AROUND_AXIS = 2

def display_reference_frame(axis, center=Vector3(), RPY=Vector3(), mut_scales=[20, 20, 20], lineWidths=[3, 3, 3],
                            colors=['r', 'g', 'b'], dim=[1, 1, 1]):
    x = Vector3(dim[0], 0, 0)
    x_rot = RotateVector(x, RPY)
    x_axis = Arrow3D([center.x, center.x + x_rot.x], [center.y, center.y + x_rot.y],
                [center.z, center.z + x_rot.z], mutation_scale=mut_scales[0],
                lw=lineWidths[0], arrowstyle="-|>", color=colors[0])

    y = Vector3(0, dim[1], 0)
    y_rot = RotateVector(y, RPY)
    y_axis = Arrow3D([center.x, center.x + y_rot.x], [center.y, center.y + y_rot.y],
                     [center.z, center.z + y_rot.z], mutation_scale=mut_scales[1],
                     lw=lineWidths[1], arrowstyle="-|>", color=colors[1])
    z = Vector3(0, 0, dim[2])
    z_rot = RotateVector(z, RPY)
    z_axis = Arrow3D([center.x, center.x + z_rot.x], [center.y, center.y + z_rot.y],
                     [center.z, center.z + z_rot.z], mutation_scale=mut_scales[2],
                     lw=lineWidths[2], arrowstyle="-|>", color=colors[2])
    axis.add_artist(x_axis)
    axis.add_artist(y_axis)
    axis.add_artist(z_axis)

def display_arrow(axis, beginning=Vector3(), ending=Vector3(), mut_scale=20, lineWidth=3,
                  color='r'):
    arrow = Arrow3D([beginning.x, ending.x], [beginning.y, ending.y],
                [beginning.z, ending.z], mutation_scale=mut_scale,
                lw=lineWidth, arrowstyle="-|>", color=color)
    axis.add_artist(arrow)

def ellipse_fitting(A, b):
    coeffs = linalg.lstsq(A, b)[0].squeeze()
    a = coeffs[0]
    b = coeffs[1]
    c = coeffs[2]
    d = coeffs[3]
    e = coeffs[4]
    f = 1
    # Ellipse check:
    isEllipse = False
    if (b ** 2 - 4 * a * c) < 0:
        isEllipse = True
        # Center:
        xc = (2 * c * d - b * e) / (b ** 2 - 4 * a * c)
        yc = (2 * a * e - b * d) / (b ** 2 - 4 * a * c)
        aa = (- math.sqrt(2 * (a * (e ** 2) + c * (d ** 2) - b * d * e + (b ** 2 - 4 * a * c) * f) * (
                a + c + math.sqrt((a - c) ** 2) + b ** 2))) / (b ** 2 - 4 * a * c)
        ab = (- math.sqrt(2 * (a * (e ** 2) + c * (d ** 2) - b * d * e + (b ** 2 - 4 * a * c) * f) * (
                    a + c - math.sqrt((a - c) ** 2) + b ** 2))) / (b ** 2 - 4 * a * c)
        if b == 0 and a < c:
            theta = 0
        elif b == 0 and a > c:
            theta = deg2rad(90)
        else:
            theta = math.atan2((c - a - math.sqrt((a - c) ** 2 + b ** 2)), b)
        return [isEllipse, xc, yc, aa, ab, b ** 2 - 4 * a * c, theta]
    return [isEllipse, b ** 2 - 4 * a * c]
# ======================================================================================================================
#                                                   S E T T I N G S
# ======================================================================================================================
# Experiment type:
experiment_type = TaskType.LINEAR_AND_ROUNDS

if experiment_type == TaskType.LINEAR_AND_ROUNDS:
    # Delays:
    takeoff_delay = 5.0
    LR_forward_displacement_delay = 5.0

    # Forward displacement along x:
    LR_forward_displacement_distance = 2.0  # [m]
    LR_forward_displacement_velocity = 0.5  # [m/s]

    # Rotation:
    LR_rotating_speed = 45.0  # [deg/s]
    LR_number_of_rotations = 1

# Time round precision:
time_round_precision = 2

# Figure to plot:
show_first_path = False
show_rotation_before = False
show_rotation_after = False

show_comparison_cm_first_path = False
show_comparison_rotation_CM = True
show_comparison_rotation_levels = False

show_traj_approximation = False

# Colors:
level0_color = '#0047AB'
level1_color = '#6495ED'
level2_color = '#0096FF'
center_color = '#FFA500'
center_path_color = '#FF6347'
# ======================================================================================================================
#                                             D A T A  E X T R A C T I O N
# ======================================================================================================================
# Determining bag name:
if experiment_type == TaskType.LINEAR_AND_ROUNDS:
    bag_file_name = 'PS_LR_N1.bag'
elif experiment_type == TaskType.ROUNDS:
    bag_file_name = 'PS_R_N1.bag'
else:
    bag_file_name = 'PS_AA_N1.bag'

# Determinig bag path:
rospack = rospkg.RosPack()
bag_path = rospack.get_path('crazyCmd') + '/data/output/Rosbags/' + bag_file_name

# Bag instance:
bag = rosbag.Bag(bag_path)

# Extracting data:
states_collection = []
for topic, msg, t in bag.read_messages(topics=['/pyramid_swarm/states']):
    states_collection.append(msg.states)
sim_secs = round(bag.get_end_time() - bag.get_start_time(), time_round_precision)
bag.close()

# Time array definition:
sim_time = linspace(0, sim_secs, len(states_collection))
number_of_cfs = len(states_collection[0])

# ======================================================================================================================
#                                                   R E A L  S I G N A L S
# ======================================================================================================================
# Center of mass actual evolution:
cm_first_path_real_x = []
cm_first_path_real_y = []
cm_first_path_real_z = []


average_yaw_angle = []

for ii in range(0, len(sim_time)):
    x_sum = 0
    y_sum = 0
    z_sum = 0
    yaw_sum = 0
    for jj in range(0, number_of_cfs):
        x_sum += states_collection[ii][jj].position.x
        y_sum += states_collection[ii][jj].position.y
        z_sum += states_collection[ii][jj].position.z
        yaw_sum += rad2deg(states_collection[ii][jj].orientation.yaw)
    cm_first_path_real_x.append(x_sum / number_of_cfs)
    cm_first_path_real_y.append(y_sum / number_of_cfs)
    cm_first_path_real_z.append(z_sum / number_of_cfs)
    average_yaw_angle.append(yaw_sum / number_of_cfs)

# Center of mass velocity:
cm_first_path_real_vx = []
cm_first_path_real_vy = []
cm_first_path_real_vz = []
dt = sim_time[1] - sim_time[0]
print('dt =', dt)
for ii in range(0, len(cm_first_path_real_z) - 1):
    cm_first_path_real_vx.append((cm_first_path_real_x[ii + 1] - cm_first_path_real_x[ii]) / dt)
    cm_first_path_real_vy.append((cm_first_path_real_y[ii + 1] - cm_first_path_real_y[ii]) / dt)
    cm_first_path_real_vz.append((cm_first_path_real_z[ii + 1] - cm_first_path_real_z[ii]) / dt)

# Center of mass position during rotation:
rotation_initial_time = LR_forward_displacement_distance / LR_forward_displacement_velocity + \
                        LR_forward_displacement_delay
rotation_final_time = rotation_initial_time + (LR_number_of_rotations * 360) / LR_rotating_speed
rotation_initial_pos = argmax(sim_time>rotation_initial_time)

rotation_final_pos = argmax(sim_time>rotation_final_time)
rotation_final_pos = len(sim_time) - 1
real_time_rotation = sim_time[rotation_initial_pos:rotation_final_pos]

# Center of mass position during rotation:
cm_rotation_real_x = []
cm_rotation_real_y = []
cm_rotation_real_z = []

for ii in range(rotation_initial_pos, rotation_final_pos):
    x_sum = 0
    y_sum = 0
    z_sum = 0
    for jj in range(0, number_of_cfs):
        x_sum += states_collection[ii][jj].position.x
        y_sum += states_collection[ii][jj].position.y
        z_sum += states_collection[ii][jj].position.z
    cm_rotation_real_x.append(x_sum/number_of_cfs)
    cm_rotation_real_y.append(y_sum / number_of_cfs)
    cm_rotation_real_z.append(z_sum / number_of_cfs)

# Real circular path level 1:
cf2_rotation_trajectory_x = []
cf3_rotation_trajectory_x = []
cf4_rotation_trajectory_x = []
cf5_rotation_trajectory_x = []
cf2_rotation_trajectory_y = []
cf3_rotation_trajectory_y = []
cf4_rotation_trajectory_y = []
cf5_rotation_trajectory_y = []

cf6_rotation_trajectory_x = []
cf6_rotation_trajectory_y = []
cf7_rotation_trajectory_x = []
cf7_rotation_trajectory_y = []
cf8_rotation_trajectory_x = []
cf8_rotation_trajectory_y = []
cf9_rotation_trajectory_x = []
cf9_rotation_trajectory_y = []
cf10_rotation_trajectory_x = []
cf10_rotation_trajectory_y = []
cf11_rotation_trajectory_x = []
cf11_rotation_trajectory_y = []
cf12_rotation_trajectory_x = []
cf12_rotation_trajectory_y = []
cf13_rotation_trajectory_x = []
cf13_rotation_trajectory_y = []
for ii in range(rotation_initial_pos, rotation_final_pos):
    for jj in range(0, number_of_cfs):
        if jj == 1:
            cf2_rotation_trajectory_x.append(states_collection[ii][jj].position.x)
            cf2_rotation_trajectory_y.append(states_collection[ii][jj].position.y)
        elif jj == 2:
            cf3_rotation_trajectory_x.append(states_collection[ii][jj].position.x)
            cf3_rotation_trajectory_y.append(states_collection[ii][jj].position.y)
        elif jj == 3:
            cf4_rotation_trajectory_x.append(states_collection[ii][jj].position.x)
            cf4_rotation_trajectory_y.append(states_collection[ii][jj].position.y)
        elif jj == 4:
            cf5_rotation_trajectory_x.append(states_collection[ii][jj].position.x)
            cf5_rotation_trajectory_y.append(states_collection[ii][jj].position.y)
        elif jj == 5:
            cf6_rotation_trajectory_x.append(states_collection[ii][jj].position.x)
            cf6_rotation_trajectory_y.append(states_collection[ii][jj].position.y)
        elif jj == 6:
            cf7_rotation_trajectory_x.append(states_collection[ii][jj].position.x)
            cf7_rotation_trajectory_y.append(states_collection[ii][jj].position.y)
        elif jj == 7:
            cf8_rotation_trajectory_x.append(states_collection[ii][jj].position.x)
            cf8_rotation_trajectory_y.append(states_collection[ii][jj].position.y)
        elif jj == 8:
            cf9_rotation_trajectory_x.append(states_collection[ii][jj].position.x)
            cf9_rotation_trajectory_y.append(states_collection[ii][jj].position.y)
        elif jj == 9:
            cf10_rotation_trajectory_x.append(states_collection[ii][jj].position.x)
            cf10_rotation_trajectory_y.append(states_collection[ii][jj].position.y)
        elif jj == 10:
            cf11_rotation_trajectory_x.append(states_collection[ii][jj].position.x)
            cf11_rotation_trajectory_y.append(states_collection[ii][jj].position.y)
        elif jj == 11:
            cf12_rotation_trajectory_x.append(states_collection[ii][jj].position.x)
            cf12_rotation_trajectory_y.append(states_collection[ii][jj].position.y)
        elif jj == 12:
            cf13_rotation_trajectory_x.append(states_collection[ii][jj].position.x)
            cf13_rotation_trajectory_y.append(states_collection[ii][jj].position.y)


# Ellipses fitting:
rotation_trajectories_level1_x = [cf2_rotation_trajectory_x, cf3_rotation_trajectory_x, cf4_rotation_trajectory_x, cf5_rotation_trajectory_x]
rotation_trajectories_level1_y = [cf2_rotation_trajectory_y, cf3_rotation_trajectory_y, cf4_rotation_trajectory_y, cf5_rotation_trajectory_y]
a_parameters = []
b_parameters = []
xcs = []
ycs = []
theta = []

for ii in range(0, len(rotation_trajectories_level1_x)):
    b = ones((len(real_time_rotation), 1)) * (-1)
    X = (array(rotation_trajectories_level1_x[ii]).T).reshape((len(real_time_rotation), 1))
    Y = (array(rotation_trajectories_level1_y[ii]).T).reshape((len(real_time_rotation), 1))
    A = hstack([X ** 2, X * Y, Y ** 2, X, Y])
    coeffs = ellipse_fitting(A, b)
    if not coeffs[0]:
        print('ERROR:  fitting is not an ellipse for cf', ii + 2)
        a_parameters.append(0)
        b_parameters.append(0)
        xcs.append(0)
        ycs.append(0)
        theta.append(0)
    else:
        a = coeffs[3]
        b = coeffs[4]
        cf_ecc = math.sqrt(1 - ((b ** 2) / (a ** 2)))
        a_parameters.append(a)
        b_parameters.append(b)
        xcs.append(coeffs[1])
        ycs.append(coeffs[2])
        theta.append(coeffs[6])
        print('cf', ii + 2, ': delta =', coeffs[5], '; a =', a, '; b =', b, '; e =', cf_ecc)

# Real circular path level 2:
positions_level2 = zeros((len(real_time_rotation), 2 * 8))
pos_for_extraction = linspace(-5, 3, 9)

for ii in range(0, len(real_time_rotation)):
    cont = 0
    for jj in range(5, 13):
        posx = int(jj + pos_for_extraction[cont])
        posy = int(posx + 1)
        positions_level2[ii, posx] = states_collection[ii+rotation_initial_pos][jj].position.x
        positions_level2[ii, posy] = states_collection[ii+rotation_initial_pos][jj].position.y
        cont += 1
print(positions_level2.shape)
# Ellipses:
cont = 0
for ii in range(5, 13):
    b = ones((len(real_time_rotation), 1)) * (-1)
    X = positions_level2[:, int(ii + pos_for_extraction[cont])].reshape((len(real_time_rotation), 1))
    Y = positions_level2[:, int(ii + pos_for_extraction[cont]) + 1].reshape((len(real_time_rotation), 1))
    A = hstack([X ** 2, X * Y, Y ** 2, X, Y])
    coeffs = ellipse_fitting(A, b)
    if not coeffs[0]:
        print('ERROR:  fitting is not an ellipse for cf', ii + 1, ', delta =', coeffs[1])
        '''a_parameters.append(0)
        b_parameters.append(0)
        xcs.append(0)
        ycs.append(0)
        theta.append(0)'''
    else:
        a = coeffs[3]
        b = coeffs[4]
        '''a_parameters.append(a)
        b_parameters.append(b)
        xcs.append(coeffs[1])
        ycs.append(coeffs[2])
        theta.append(coeffs[6])'''
        cf_ecc = math.sqrt(1 - ((b ** 2) / (a ** 2)))
        #print('cf', ii + 1, ': delta =', coeffs[5], '; a =', a, '; b =', b, '; e =', cf_ecc)


rotation_trajectories_level2_x = [cf6_rotation_trajectory_x, cf7_rotation_trajectory_x, cf8_rotation_trajectory_x,
                                  cf9_rotation_trajectory_x, cf10_rotation_trajectory_x, cf11_rotation_trajectory_x,
                                  cf12_rotation_trajectory_x, cf13_rotation_trajectory_x]
rotation_trajectories_level2_y = [cf6_rotation_trajectory_y, cf7_rotation_trajectory_y, cf8_rotation_trajectory_y,
                                  cf9_rotation_trajectory_y, cf10_rotation_trajectory_y, cf11_rotation_trajectory_y,
                                  cf12_rotation_trajectory_y, cf13_rotation_trajectory_y]
for ii in range(0, len(rotation_trajectories_level2_x)):
    b = ones((len(real_time_rotation), 1)) * (-1)
    X = (array(rotation_trajectories_level2_x[ii]).T).reshape((len(real_time_rotation), 1))
    Y = (array(rotation_trajectories_level2_y[ii]).T).reshape((len(real_time_rotation), 1))
    A = hstack([X ** 2, X * Y, Y ** 2, X, Y])
    coeffs = ellipse_fitting(A, b)
    if not coeffs[0]:
        print('ERROR:  fitting is not an ellipse for cf', ii + 2)
        a_parameters.append(0)
        b_parameters.append(0)
        xcs.append(0)
        ycs.append(0)
        theta.append(0)
    else:
        a = coeffs[3]
        b = coeffs[4]
        cf_ecc = math.sqrt(1 - ((b ** 2) / (a ** 2)))
        a_parameters.append(a)
        b_parameters.append(b)
        xcs.append(coeffs[1])
        ycs.append(coeffs[2])
        theta.append(coeffs[6])
        print('cf', ii + 6, ': delta =', coeffs[5], '; a =', a, '; b =', b, '; e =', cf_ecc)

# Calculating distorsion:

# ======================================================================================================================
#                                                   R E F E R E N C E
# ======================================================================================================================
if experiment_type == TaskType.LINEAR_AND_ROUNDS:
    # Initial conditions (once takeoff ended):
    initial_conditions = [[0, 0, 1.5],
                          [-0.5, -0.5, 1.0],
                          [-0.5, 0.5, 1.0],
                          [0.5, 0.5, 1.0],
                          [0.5, -0.5, 1.0],
                          [-1, -1, 0.5],
                          [-1, 0, 0.5],
                          [-1, 1, 0.5],
                          [0, 1, 0.5],
                          [1, 1, 0.5],
                          [1, 0, 0.5],
                          [1, -1, 0.5],
                          [0, -1, 0.5]]
    x0 = []
    y0 = []
    z0 = []
    colors = []
    # Filling initial condition and find initial center of mass:
    x_sum = 0
    y_sum = 0
    z_sum = 0
    for ii in range(0, len(initial_conditions)):
        x0.append(initial_conditions[ii][0])
        y0.append(initial_conditions[ii][1])
        z0.append(initial_conditions[ii][2])
        x_sum += x0[-1]
        y_sum += y0[-1]
        z_sum += z0[-1]
        if ii == 0:
            colors.append(level0_color)
        elif ii <= 6:
            colors.append(level1_color)
        else:
            colors.append(level2_color)

    # Finding initial center of mass:
    cm_0 = [x_sum/number_of_cfs, y_sum/number_of_cfs, z_sum/number_of_cfs]
    print('INITIAL CENTER OF MASS:', cm_0)
    # First path ideal center of mass:
    th_time_first_path = LR_forward_displacement_distance / LR_forward_displacement_velocity
    time_first_path = linspace(0, th_time_first_path, len(sim_time[0:argmax(sim_time>th_time_first_path)]))
    cm_position_first_path_ref_x = []
    cm_position_first_path_ref_y = []
    cm_position_first_path_ref_z = []
    dt = time_first_path[1] - time_first_path[0]
    prev_val = cm_0[0]
    first_path_z0 = cm_0[2]
    for ii in range(0, len(time_first_path)):
        cm_position_first_path_ref_x.append(prev_val + LR_forward_displacement_velocity * dt)
        cm_position_first_path_ref_y.append(0.0)
        cm_position_first_path_ref_z.append(first_path_z0)
        prev_val = cm_position_first_path_ref_x[-1]

    # Velocity forward path:
    cm_first_path_ref_vx = ones((len(time_first_path)-1,)) * LR_forward_displacement_velocity
    cm_first_path_ref_vy = zeros((len(time_first_path)-1,))
    cm_first_path_ref_vz = zeros((len(time_first_path)-1,))

    # CM position during rotation:
    cm_rotation_ref_x = ones((len(real_time_rotation,))) * LR_forward_displacement_distance
    cm_rotation_ref_y = zeros((len(real_time_rotation,)))
    cm_rotation_ref_z = ones((len(real_time_rotation, ))) * cm_position_first_path_ref_z[-1]

    # Level 1 reference:
    level1_ref_circle1 = []
    level1_ref_circle2 = []
    level1_ref_x = linspace(1.5, 2.5, len(real_time_rotation))
    for ii in range(0, len(real_time_rotation)):
        level1_ref_circle1.append(math.sqrt(0.5 ** 2 - (level1_ref_x[ii] - 2) ** 2))
        level1_ref_circle2.append(- math.sqrt(0.5 ** 2 - (level1_ref_x[ii] - 2) ** 2))

    # Level 2 reference external:
    level2_ref_circle1_ext = []
    level2_ref_circle2_ext = []
    level2_ref_x_ext = linspace(1.0, 3.0, len(real_time_rotation))
    for ii in range(0, len(real_time_rotation)):
        level2_ref_circle1_ext.append(math.sqrt(1.0 ** 2 - (level2_ref_x_ext[ii] - 2) ** 2))
        level2_ref_circle2_ext.append(- math.sqrt(1.0 ** 2 - (level2_ref_x_ext[ii] - 2) ** 2))

    # Level 2 reference internal:
    level2_ref_circle1_int = []
    level2_ref_circle2_int = []
    r_int = abs(math.sin(math.pi / 4))
    level2_ref_x_int = linspace(2 - r_int, 2 + r_int, len(real_time_rotation))

    for ii in range(0, len(real_time_rotation)):
        level2_ref_circle1_int.append(math.sqrt(r_int ** 2 - (level2_ref_x_int[ii] - 2) ** 2))
        level2_ref_circle2_int.append(- math.sqrt(r_int ** 2 - (level2_ref_x_int[ii] - 2) ** 2))


# ======================================================================================================================
#                                                   F I G U R E S
# ======================================================================================================================
fig_cont = 0
# Reference trajectory
if show_first_path:
    fig_cont += 1
    figure(fig_cont)
    ax = axes(projection='3d')
    for ii in range(0, len(x0)):
        ax.scatter3D(x0[ii], y0[ii], z0[ii], color=colors[ii])
    ax.scatter3D(cm_0[0], cm_0[1], cm_0[2], color=center_color)
    ax.scatter3D(cm_position_first_path_ref_x[-1], cm_position_first_path_ref_y[-1], cm_position_first_path_ref_z[-1],
                 color=center_color)
    display_arrow(ax, Vector3(cm_0[0], cm_0[1], cm_0[2]), Vector3(cm_position_first_path_ref_x[-1],
                                                                  cm_position_first_path_ref_y[-1],
                                                                  cm_position_first_path_ref_z[-1]), color='#DB7093')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title('Forward path')

    ax.set_xlim3d(left=-1, right=4)
    ax.set_ylim3d(bottom=-3, top=3)
    ax.set_zlim3d(bottom=0, top=2)

if show_rotation_before:
    fig_cont += 1
    figure(fig_cont)
    ax = axes(projection='3d')
    for ii in range(0, len(x0)):
        ax.scatter3D(x0[ii]+2, y0[ii], z0[ii], color=colors[ii])

    ax.scatter3D(cm_position_first_path_ref_x[-1], cm_position_first_path_ref_y[-1], cm_position_first_path_ref_z[-1],
                 color=center_color)

    display_reference_frame(ax, Vector3(cm_position_first_path_ref_x[-1], cm_position_first_path_ref_y[-1],
                                        cm_position_first_path_ref_z[-1]), Vector3(0, 0, deg2rad(0)), dim=[0.75, 0.75, 0.3],
                            mut_scales=[5, 5, 5])
    display_arrow(ax, Vector3(cm_position_first_path_ref_x[-1], cm_position_first_path_ref_y[-1], 1.5),
                  Vector3(cm_position_first_path_ref_x[-1], cm_position_first_path_ref_y[-1], 2),
                  color='#DB7093')
    display_arrow(ax, Vector3(cm_position_first_path_ref_x[-1], cm_position_first_path_ref_y[-1], 1.5),
                  Vector3(cm_position_first_path_ref_x[-1], cm_position_first_path_ref_y[-1], 1.9),
                  color='#DB7093')


    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title('Rotation')

    ax.set_xlim3d(left=-1, right=4)
    ax.set_ylim3d(bottom=-3, top=3)
    ax.set_zlim3d(bottom=0, top=2)

if show_rotation_after:
    fig_cont += 1
    figure(fig_cont)
    ax = axes(projection='3d')
    for ii in range(0, len(x0)):
        ax.scatter3D(x0[ii] + 2, y0[ii], z0[ii], color=colors[ii])

    ax.scatter3D(cm_position_first_path_ref_x[-1], cm_position_first_path_ref_y[-1], cm_position_first_path_ref_z[-1],
                 color=center_color)

    display_reference_frame(ax, Vector3(cm_position_first_path_ref_x[-1], cm_position_first_path_ref_y[-1],
                                        cm_position_first_path_ref_z[-1]), Vector3(0, 0, deg2rad(0)), dim=[0.75, 0.75, 0.3],
                            mut_scales=[5, 5, 5])
    display_arrow(ax, Vector3(cm_position_first_path_ref_x[-1], cm_position_first_path_ref_y[-1], 1.5),
                  Vector3(cm_position_first_path_ref_x[-1], cm_position_first_path_ref_y[-1], 2),
                  color='#DB7093')
    display_arrow(ax, Vector3(cm_position_first_path_ref_x[-1], cm_position_first_path_ref_y[-1], 1.5),
                  Vector3(cm_position_first_path_ref_x[-1], cm_position_first_path_ref_y[-1], 1.9),
                  color='#DB7093')

    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title('End of rotation')

    ax.set_xlim3d(left=-1, right=4)
    ax.set_ylim3d(bottom=-3, top=3)
    ax.set_zlim3d(bottom=0, top=2)


if show_comparison_cm_first_path:
    fig_cont += 1
    figure(fig_cont)
    # X POSITION:
    subplot(311)
    plot(time_first_path, cm_position_first_path_ref_x, label='Ref')
    plot(sim_time[0:argmax(sim_time>th_time_first_path)], cm_first_path_real_x[0:argmax(sim_time>th_time_first_path)], label='Real')
    ylabel('X [m]')
    title('CM forward path: position comparison')
    legend(loc="upper right")

    # Y POSITION:
    subplot(312)
    plot(time_first_path, cm_position_first_path_ref_y, label='Ref')
    plot(sim_time[0:argmax(sim_time>th_time_first_path)], cm_first_path_real_y[0:argmax(sim_time>th_time_first_path)], label='Real')
    ylabel('Y [m]')
    legend(loc="upper right")

    # Z POSITION:
    subplot(313)
    plot(time_first_path, cm_position_first_path_ref_z, label='Ref')
    plot(sim_time[0:argmax(sim_time>th_time_first_path)], cm_first_path_real_z[0:argmax(sim_time>th_time_first_path)], label='Real')
    ylabel('Z [m]')
    legend(loc="upper right")

    fig_cont += 1
    figure(fig_cont)
    # Vx:
    subplot(311)
    plot(time_first_path[0:-1], cm_first_path_ref_vx, label='Ref')
    '''plot(sim_time[0:argmax(sim_time > th_time_first_path)-1],
         cm_first_path_real_vx[0:argmax(sim_time > th_time_first_path)-1], label='Real')'''
    plot(sim_time[0:argmax(sim_time > th_time_first_path) - 1],
         savgol_filter(cm_first_path_real_vx[0:argmax(sim_time > th_time_first_path) - 1], 51, 2), label='Real')
    ylabel('Vx [m/s]')
    title('CM forward path: velocity comparison')
    legend(loc="upper right")

    # Vy:
    subplot(312)
    plot(time_first_path[0:-1], cm_first_path_ref_vy, label='Ref')
    '''plot(sim_time[0:argmax(sim_time > th_time_first_path)-1],
         cm_first_path_real_vy[0:argmax(sim_time > th_time_first_path)-1], label='Real')'''
    plot(sim_time[0:argmax(sim_time > th_time_first_path) - 1],
         savgol_filter(cm_first_path_real_vy[0:argmax(sim_time > th_time_first_path) - 1], 51, 2), label='Real')
    ylabel('Vy [m/s]')
    legend(loc="upper right")

    # Vy:
    subplot(313)
    plot(time_first_path[0:-1], cm_first_path_ref_vz, label='Ref')
    '''plot(sim_time[0:argmax(sim_time > th_time_first_path)-1],
         cm_first_path_real_vz[0:argmax(sim_time > th_time_first_path)-1], label='Real')'''
    plot(sim_time[0:argmax(sim_time > th_time_first_path) - 1],
         savgol_filter(cm_first_path_real_vz[0:argmax(sim_time > th_time_first_path) - 1], 51, 5), label='Real')
    ylabel('Vz [m/s]')
    legend(loc="upper right")

    # Yaw
    fig_cont += 1
    figure(fig_cont)
    plot(sim_time[0:argmax(sim_time > th_time_first_path)],
         average_yaw_angle[0:argmax(sim_time > th_time_first_path)], label='Real')
    ylabel('Yaw [deg]')
    xlabel('Time [s]')
    title('Average yaw orientation')
    print('Average yaw value during forward path: ', mean(average_yaw_angle[0:argmax(sim_time > th_time_first_path)]), '[deg]')

if show_comparison_rotation_CM:
    fig_cont += 1
    figure(fig_cont)
    # X POSITION:
    subplot(311)
    plot(real_time_rotation, cm_rotation_ref_x, label='Ref')
    plot(real_time_rotation, cm_rotation_real_x, label='Real')
    ylabel('X [m]')
    title('CM during rotation: position comparison')
    legend(loc="upper right")

    # Y POSITION:
    subplot(312)
    plot(real_time_rotation, cm_rotation_ref_y, label='Ref')
    plot(real_time_rotation, cm_rotation_real_y, label='Real')
    ylabel('Y [m]')
    legend(loc="upper right")

    # Z POSITION:
    subplot(313)
    plot(real_time_rotation, cm_rotation_ref_z, label='Ref')
    plot(real_time_rotation, cm_rotation_real_z, label='Real')
    ylabel('Z [m]')
    legend(loc="upper right")

    fig_cont += 1
    figure(fig_cont)
    # Correlation:
    subplot(311)
    cross_corr_x = correlate(cm_rotation_ref_x, cm_rotation_real_x)
    auto_corr_ref_x = correlate(cm_rotation_ref_x, cm_rotation_ref_x)

    cross_corr_z = correlate(cm_rotation_ref_z, cm_rotation_real_z)
    auto_corr_ref_z = correlate(cm_rotation_ref_z, cm_rotation_ref_z)

    rel_corr_x = []
    rel_corr_y = []
    rel_corr_z = []
    for ii in range(0, len(cross_corr_x)):
        rel_corr_x.append(cross_corr_x[ii] / auto_corr_ref_x[ii])
        rel_corr_z.append(cross_corr_z[ii] / auto_corr_ref_z[ii])
    real_time_rotation_to0 = real_time_rotation - real_time_rotation[0]
    time_corr = concatenate((flip(real_time_rotation_to0[0:-1]) * (-1), real_time_rotation_to0))
    plot(time_corr, rel_corr_x, label='Cross X')
    plot(time_corr, rel_corr_z, label='Cross Z')
    #ylim(0.8, 1.2)
    ylabel('Corr.')
    title('CM during rotation: comparison')
    legend(loc="upper right")

    # Absolute distance:
    subplot(312)
    RMS_x_diff = 0
    RMS_y_diff = 0
    RMS_z_diff = 0

    for ii in range(0, len(real_time_rotation)):
        RMS_x_diff += (cm_rotation_ref_x[ii] - cm_rotation_real_x[ii]) ** 2
        RMS_y_diff += (cm_rotation_ref_y[ii] - cm_rotation_real_y[ii]) ** 2
        RMS_z_diff += (cm_rotation_ref_z[ii] - cm_rotation_real_z[ii]) ** 2


    RMS_x_diff = math.sqrt(RMS_x_diff / len(real_time_rotation))
    RMS_y_diff = math.sqrt(RMS_y_diff / len(real_time_rotation))
    RMS_z_diff = math.sqrt(RMS_z_diff / len(real_time_rotation))

    print('RMS delta x: ', RMS_x_diff, 'RMS delta y: ', RMS_y_diff, 'RMS delta z: ', RMS_z_diff)

    # Absolute difference:
    delta_x = []
    delta_y = []
    delta_z = []

    for ii in range(0, len(real_time_rotation)):
        delta_x.append(cm_rotation_real_x[ii]-cm_rotation_ref_x[ii])
        delta_y.append(cm_rotation_real_y[ii] - cm_rotation_ref_y[ii])
        delta_z.append(cm_rotation_real_z[ii] - cm_rotation_ref_z[ii])

    plot(real_time_rotation, savgol_filter(delta_x, 51, 2), label='Diff x')
    plot(real_time_rotation, savgol_filter(delta_y, 51, 2), label='Diff y')
    plot(real_time_rotation, delta_z, label='Diff z')
    ylabel('Abs. Diff. [m]')
    legend(loc="upper right")
    xlabel('Time [s]')

    subplot(313)
    plot(real_time_rotation, savgol_filter(delta_x, 51, 2) / 0.09, label='Diff x')
    plot(real_time_rotation, savgol_filter(delta_y, 51, 2) / 0.09, label='Diff y')
    plot(real_time_rotation, array(delta_z) / 0.09, label='Diff z')
    ylabel('Rel. Diff.')
    legend(loc="upper right")
    xlabel('Time [s]')


if show_comparison_rotation_levels:
    fig_cont += 1
    figure(fig_cont)
    # Reference path:
    plot(level1_ref_x, level1_ref_circle1, color=level1_color, label='Ref')
    plot(level1_ref_x, level1_ref_circle2, color=level1_color)
    plot(cf2_rotation_trajectory_x, cf2_rotation_trajectory_y, label='Cf2')
    plot(cf3_rotation_trajectory_x, cf3_rotation_trajectory_y, label='Cf3')
    plot(cf4_rotation_trajectory_x, cf4_rotation_trajectory_y, label='Cf4')
    plot(cf5_rotation_trajectory_x, cf5_rotation_trajectory_y, label='Cf5')
    xlabel('X [m]')
    ylabel('Y [m]')
    title('Trajectories level 1')
    axis('equal')
    xlim(1, 3)
    ylim(-1, 1)
    legend(loc="upper right")

    fig_cont += 1
    figure(fig_cont)
    # Reference path:
    plot(level1_ref_x, level1_ref_circle1, color=level1_color, label='Ref')
    plot(level1_ref_x, level1_ref_circle2, color=level1_color)
    plot(positions_level2[:, 0], positions_level2[:, 1], label='Cf6')
    print(positions_level2[:, 0].shape)
    plot(cf6_rotation_trajectory_x, cf6_rotation_trajectory_y, label='Ex')
    xlabel('X [m]')
    ylabel('Y [m]')
    title('Trajectories level 2')
    axis('equal')
    xlim(-1, 3)
    ylim(-2, 2)
    legend(loc="upper right")

if show_traj_approximation:
    fig_cont += 1
    #figure(fig_cont)
    # Cf2:
    print(len(xcs))
    fig, ax = subplots(subplot_kw={'aspect': 'equal'})

    plot(level2_ref_x_int, level2_ref_circle1_int, color=level1_color, label='Ref')
    plot(level2_ref_x_int, level2_ref_circle2_int, color=level1_color)
    ellipse = Ellipse((xcs[10], ycs[10]), (a_parameters[10]) * 2, b_parameters[10] * 2, angle=rad2deg(theta[10]), fill=False, color=center_color, linewidth=2)
    ax.add_artist(ellipse)
    plot(positions_level2[:, 12], positions_level2[:, 13], label='Real', color='g')
    plot(cf13_rotation_trajectory_x, cf13_rotation_trajectory_y, label='Real', color='r')
    legend(loc="upper right")
    title('Trajectory cf13')
    axis('equal')
    xlabel('X [m]')
    ylabel('Y [m]')
    ax.set_xlim(0, 4)
    ax.set_ylim(-3, 3)

show()
