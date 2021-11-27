import rosbag
import rospkg
from crazy_common_py.common_functions import rad2deg
from crazy_common_py.default_topics import DEFAULT_CF_STATE_TOPIC, DEFAULT_MOTOR_CMD_TOPIC, \
    DEFAULT_ACTUAL_DESTINATION_TOPIC
from matplotlib.pyplot import plot, xlabel, ylabel, show, figure, title, ylim, subplot, xlim, legend
from numpy import linspace, full, array, ones, zeros, concatenate, argmax
from scipy.signal import savgol_filter, correlate
rospack = rospkg.RosPack()

# ======================================================================================================================
#                                                   S E T T I N G S
# ======================================================================================================================

show_all_real = False
show_all_sim = False
show_all_comparison = True

# Real figures:
if show_all_real:
    show_real_position = True
    show_real_velocity = True
    show_real_orientation = True
    show_real_motor_command = True
else:
    show_real_position = False
    show_real_velocity = False
    show_real_orientation = False
    show_real_motor_command = False

# Real bags:
'''
    'real_state_R_D_04_02_04_02_V_02_02_02_02_RA_90_RR_72_N1.bag'
    'S_D_02_02_02_02_V_02_02_02_02_RA_90_RR_72_N1'
'''
real_state_bag_name = 'real_state_S_D_02_02_02_02_V_02_02_02_02_RA_90_RR_72_N1.bag'
real_motor_bag_name = 'real_motor_S_D_02_02_02_02_V_02_02_02_02_RA_90_RR_72_N1.bag'
real_ref_bag_name = 'real_ref_S_D_02_02_02_02_V_02_02_02_02_RA_90_RR_72_N1.bag'

# Simulation figures:
if show_all_sim:
    show_sim_position = True
    show_sim_velocity = True
    show_sim_orientation = True
    show_sim_motor_command = True
else:
    show_sim_position = False
    show_sim_velocity = False
    show_sim_orientation = False
    show_sim_motor_command = False

# Simulation bas:
sim_state_bag_name = 'sim_state_S_D_02_02_02_02_V_02_02_02_02_RA_90_RR_72_N1.bag'
sim_motor_bag_name = 'sim_motor_S_D_02_02_02_02_V_02_02_02_02_RA_90_RR_72_N1.bag'
sim_ref_bag_name = 'sim_ref_S_D_02_02_02_02_V_02_02_02_02_RA_90_RR_72_N1.bag'

# Comparison figures:
if show_all_comparison:
    show_comp_position = True
    show_comp_velocity = True
    show_comp_orientation = True
    show_comp_motor_command = True
else:
    show_comp_position = False
    show_comp_velocity = False
    show_comp_orientation = False
    show_comp_motor_command = False

time_round_precision = 1
fig_cont = 0

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#                                       R E F E R E N C E
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# From takeoff:
time_initial_takeoff = 2.7
time_final_takeoff = 3.6
delay_takeoff = 1.0

time_turn = 1.25
delay_turn = 1.0

time_side1_x = 1.0
delay_side1_x = 1.0

time_side2_y = 1.0
delay_side2_y = 1.0

time_side3_x = 1.0
delay_side3_x = 1.0

time_side4_y = 1.0
delay_side4_y = 5.0
# ======================================================================================================================
#                                   S I M U L A T E D  D A T A  E X T R A C T I O N
# ======================================================================================================================
# Rosbags path definitions:
sim_state_bag_path = rospack.get_path('crazyCmd') + '/data/output/Rosbags/' + sim_state_bag_name
sim_motor_bag_path = rospack.get_path('crazyCmd') + '/data/output/Rosbags/' + sim_motor_bag_name
sim_ref_bag_path = rospack.get_path('crazyCmd') + '/data/output/Rosbags/' + sim_ref_bag_name

# Rosbags instances:
sim_state_bag = rosbag.Bag(sim_state_bag_path)
sim_motor_bag = rosbag.Bag(sim_motor_bag_path)
sim_ref_bag = rosbag.Bag(sim_ref_bag_path)

# ----------------------------------------------------------------------------------------------------------------------
#                                    L I S T S  D E F I N I T I O N S
# ----------------------------------------------------------------------------------------------------------------------
# Sim angles:
sim_roll = []
sim_pitch = []
sim_yaw = []

# Sim position:
sim_x = []
sim_y = []
sim_z = []

# Sim linear vel:
sim_vx = []
sim_vy = []
sim_vz = []

# Sim rot speed:
sim_wx = []
sim_wy = []
sim_wz = []

# Sim motor commands:
sim_roll_cmd = []
sim_pitch_cmd = []
sim_yaw_cmd = []
sim_thrust_cmd = []

# Sim ref:
sim_ref_x = []
sim_ref_y = []
sim_ref_z = []

sim_ref_vx = []
sim_rev_vy = []
sim_ref_vz = []

sim_ref_wz = []
# ----------------------------------------------------------------------------------------------------------------------
#                                           D A T A  E X T R A C T I O N
# ----------------------------------------------------------------------------------------------------------------------
# Extracting data about state:
for topic, msg, t in sim_state_bag.read_messages(topics=['/cf1/'+DEFAULT_CF_STATE_TOPIC]):
    # Simulated position:
    sim_x.append(msg.position.x)
    sim_y.append(msg.position.y)
    sim_z.append(msg.position.z)

    # Sim orientation:
    sim_roll.append(rad2deg(msg.orientation.roll))
    sim_pitch.append(rad2deg(msg.orientation.pitch))
    sim_yaw.append(rad2deg(msg.orientation.yaw))

    # Sim velocity:
    sim_vx.append(msg.velocity.x)
    sim_vy.append(msg.velocity.y)
    sim_vz.append(msg.velocity.z)

    # Sim rotating speed:
    sim_wx.append(rad2deg(msg.rotating_speed.z))
    sim_wy.append(rad2deg(msg.rotating_speed.y))
    sim_wz.append(rad2deg(msg.rotating_speed.z))

sim_state_secs = round(sim_state_bag.get_end_time() - sim_state_bag.get_start_time(), time_round_precision)
sim_state_bag.close()

# Extracting data about motor commands:
for topic, msg, t in sim_motor_bag.read_messages(topics=['/cf1/'+DEFAULT_MOTOR_CMD_TOPIC]):
    # Sim motor commands:
    sim_roll_cmd.append(msg.desired_attitude.roll)
    sim_pitch_cmd.append(msg.desired_attitude.pitch)
    sim_yaw_cmd.append(msg.desired_attitude.yaw)
    sim_thrust_cmd.append(msg.desired_thrust)

sim_motor_secs = round(sim_motor_bag.get_end_time() - sim_motor_bag.get_start_time(), time_round_precision)
sim_motor_bag.close()

# Extracting data about reference:
for topic, msg, t in sim_ref_bag.read_messages(topics=['/cf1/'+DEFAULT_ACTUAL_DESTINATION_TOPIC]):
    # Sim ref velocity:
    sim_ref_vx.append(msg.desired_velocity.x)
    sim_rev_vy.append(msg.desired_velocity.y)
    sim_ref_vz.append(msg.desired_velocity.z)

    # Sim ref angular speed:
    sim_ref_wz.append(msg.desired_yaw_rate)

sim_ref_secs = round(sim_ref_bag.get_end_time() - sim_ref_bag.get_start_time(), time_round_precision)
sim_ref_bag.close()



# ----------------------------------------------------------------------------------------------------------------------
#                                                   F I G U R E S
# ----------------------------------------------------------------------------------------------------------------------
sim_time_state = linspace(0.0, sim_ref_secs, len(sim_x))
sim_time_motor = linspace(0.0, sim_motor_secs, len(sim_roll_cmd))

for ii in range(0, len(sim_time_state)):
    sim_time_state[ii] = sim_time_state[ii] + time_final_takeoff - 0.8

sim_time_state_list = list(sim_time_state)
pos = [n for n, i in enumerate(sim_time_state_list) if i > 20.0][0]

delta_time = 1.0
sim_fake_time_state = sim_time_state_list[0:pos+1]
delta_fake_list = list(linspace(sim_time_state_list[pos], sim_time_state_list[pos] + delta_time,
                                int(delta_time / (sim_time_state_list[1]-sim_time_state_list[0])) ))

sim_fake_time_state = sim_fake_time_state + delta_fake_list
sim_fake_time_tail = []
for ii in range(0, len(sim_time_state_list[pos+2:])):
    sim_fake_time_tail.append(sim_time_state_list[pos+2+ii]+delta_time)
sim_fake_time_state = sim_fake_time_state + sim_fake_time_tail

sim_z_list = list(sim_z)
sim_fake_z = sim_z_list[0:pos+1]
'''delta_fake_z_list = list(full((1, len(delta_fake_list)), 10))
for ii in range(0, len(delta_fake_z_list)):
    delta_fake_z_list[ii] = 0.3'''
height = sim_z_list[pos]
delta_fake_z_list = [height for x in range(0, len(delta_fake_list))]
sim_fake_z = sim_fake_z + delta_fake_z_list
sim_fake_z_tail = []

sim_fake_z = sim_fake_z + sim_z_list[pos+2:]


# Simulate position figure:
if show_sim_position:
    fig_cont += 1
    figure(fig_cont)

    # X position:
    subplot(311)
    plot(sim_time_state, sim_x, label='Real')
    ylabel('X [m]')
    title('Simulated position')
    legend(loc="upper right")

    # Y position:
    subplot(312)
    plot(sim_time_state, sim_y, label='Real')
    ylabel('Y [m]')
    legend(loc="upper right")

    # Z position:
    subplot(313)
    plot(sim_time_state, sim_z, label='Real')
    ylabel('Z [m]')
    xlabel('Time [s]')
    legend(loc="upper right")


# Simulated velocity figure:
if show_sim_velocity:
    fig_cont += 1
    figure(fig_cont)

    # Sim Vx:
    subplot(411)
    plot(sim_time_state, sim_vx, label='Real')
    ylabel('Vx [m/s]')
    title('Simulated velocity')
    legend(loc="upper right")

    # Sim Vy:
    subplot(412)
    plot(sim_time_state, sim_vy, label='Real')
    ylabel('Vy [m/s]')
    legend(loc="upper right")

    # Sim Vz:
    subplot(413)
    plot(sim_time_state, sim_vz, label='Real')
    ylabel('Vz [m/s]')
    xlabel('Time [s]')
    legend(loc="upper right")

    # Sim Wz:
    subplot(414)
    plot(sim_time_state, sim_wz, label='Real')
    ylabel('Wz [deg/s]')
    xlabel('Time [s]')
    legend(loc="upper right")

if show_sim_orientation:
    fig_cont += 1
    figure(fig_cont)

    # Sim roll:
    subplot(311)
    plot(sim_time_state, sim_roll, label='Real')
    ylabel('R [deg]')
    title('Simulated orientation')
    legend(loc="upper right")

    # Sim pitch:
    subplot(312)
    plot(sim_time_state, sim_pitch, label='Real')
    ylabel('P [deg]')
    legend(loc="upper right")

    # Sim yaw:
    subplot(313)
    plot(sim_time_state, sim_yaw, label='Real')
    ylabel('Y [deg]')
    xlabel('Time [s]')
    legend(loc="upper right")

if show_sim_motor_command:
    fig_cont += 1
    figure(fig_cont)

    # Sim roll command:
    subplot(411)
    plot(sim_time_motor, sim_roll_cmd, label='Real')
    ylabel('Rc [u]')
    title('Simulated motor commands')
    legend(loc="upper right")

    # Sim pitch command:
    subplot(412)
    plot(sim_time_motor, sim_pitch_cmd, label='Real')
    ylabel('Pc [u]')
    legend(loc="upper right")

    # Sim yaw command:
    subplot(413)
    plot(sim_time_motor, sim_yaw_cmd, label='Real')
    ylabel('Yc [u]')
    xlabel('Time [s]')
    legend(loc="upper right")

    # Sim thrust command:
    subplot(414)
    plot(sim_time_motor, sim_thrust_cmd, label='Real')
    ylabel('Tc [u]')
    xlabel('Time [s]')
    legend(loc="upper right")



# ======================================================================================================================
#                                   R E A L  D A T A  E X T R A C T I O N
# ======================================================================================================================
# Rosbags path definitions:
real_state_bag_path = rospack.get_path('crazyCmd') + '/data/output/Rosbags/' + real_state_bag_name
real_motor_bag_path = rospack.get_path('crazyCmd') + '/data/output/Rosbags/' + real_motor_bag_name
real_ref_bag_path = rospack.get_path('crazyCmd') + '/data/output/Rosbags/' + real_ref_bag_name

# Rosbags instances:
real_state_bag = rosbag.Bag(real_state_bag_path)
real_motor_bag = rosbag.Bag(real_motor_bag_path)
real_ref_bag = rosbag.Bag(real_ref_bag_path)

# ----------------------------------------------------------------------------------------------------------------------
#                                    L I S T S  D E F I N I T I O N S
# ----------------------------------------------------------------------------------------------------------------------
# Real angles:
real_roll = []
real_pitch = []
real_yaw = []

# Real position:
real_x = []
real_y = []
real_z = []

# Real linear vel:
real_vx = []
real_vy = []
real_vz = []

# Real rotating speed:
real_wx = []
real_wy = []
real_wz = []

# Real motor commands:
real_roll_cmd = []
real_pitch_cmd = []
real_yaw_cmd = []
real_thrust_cmd = []

# Real reference:
real_ref_x = []
real_ref_y = []
real_ref_z = []

real_ref_vx = []
real_ref_vy = []
real_ref_vz = []

real_ref_wz = []


# ----------------------------------------------------------------------------------------------------------------------
#                                           D A T A  E X T R A C T I O N
# ----------------------------------------------------------------------------------------------------------------------
# Extracting real state:
for topic, msg, t in real_state_bag.read_messages(topics=['/cf1/'+DEFAULT_CF_STATE_TOPIC]):
    # Real position:
    real_x.append(msg.position.x)
    real_y.append(msg.position.y)
    real_z.append(msg.position.z)

    # Real orientation:
    real_roll.append(rad2deg(msg.orientation.roll))
    real_pitch.append(rad2deg(msg.orientation.pitch))
    real_yaw.append(rad2deg(msg.orientation.yaw))

    # Real velocity:
    real_vx.append(msg.velocity.x)
    real_vy.append(msg.velocity.y)
    real_vz.append(msg.velocity.z)

    # Real rotating speed:
    real_wx.append(rad2deg(msg.rotating_speed.x))
    real_wy.append(rad2deg(msg.rotating_speed.y))
    real_wz.append(rad2deg(msg.rotating_speed.z))

real_state_secs = round(real_state_bag.get_end_time() - real_state_bag.get_start_time(), time_round_precision)
real_state_bag.close()

# Extracting real motor command:
for topic, msg, t in real_motor_bag.read_messages(topics=['/cf1/'+DEFAULT_MOTOR_CMD_TOPIC]):
    # Real motor commands:
    real_roll_cmd.append(msg.desired_attitude.roll)
    real_pitch_cmd.append(msg.desired_attitude.pitch)
    real_yaw_cmd.append(msg.desired_attitude.yaw)
    real_thrust_cmd.append(msg.desired_thrust)

real_motor_secs = round(real_motor_bag.get_end_time() - real_motor_bag.get_start_time(), time_round_precision)
real_motor_bag.close()

# Extracting real reference:
for topic, msg, t in real_ref_bag.read_messages(topics=['/cf1/'+DEFAULT_ACTUAL_DESTINATION_TOPIC]):
    # Real velocity references:
    real_ref_vx.append(msg.desired_velocity.x)
    real_ref_vy.append(msg.desired_velocity.y)
    real_ref_vz.append(msg.desired_velocity.z)

    # Real yaw rate reference:
    real_ref_wz.append(msg.desired_yaw_rate)

real_ref_secs = round(real_ref_bag.get_end_time() - real_ref_bag.get_start_time(), time_round_precision)
real_ref_bag.close()

# ----------------------------------------------------------------------------------------------------------------------
#                                                   F I G U R E S
# ----------------------------------------------------------------------------------------------------------------------
# Time lists
real_time_state = linspace(0.0, real_ref_secs, len(real_x))
real_time_motor = linspace(0.0, real_motor_secs, len(real_roll_cmd))
real_time_ref = linspace(0.0, real_ref_secs, len(real_ref_vx))

# Building theoretical x position:
x0 = 0
dt = real_time_ref[1]-real_time_ref[0]
first = True

start_time_side1_x = time_final_takeoff + delay_takeoff                # 4.66
end_time_side1_x = start_time_side1_x + time_side1_x
start_time_side3_x = end_time_side1_x + delay_side1_x + time_turn + delay_turn + time_side2_y + delay_side2_y + time_turn + delay_turn    # 14.21
end_time_side3_x = start_time_side3_x + time_side3_x

key_times = [start_time_side1_x, end_time_side1_x, start_time_side3_x, end_time_side3_x, 500]
pos = 0
increment = False
v = 0.2
for ii in range(0, len(real_ref_vx)):
    if real_time_ref[ii] >= key_times[pos]:
        pos += 1
        increment = not increment
    if pos == 2:
        v = - 0.2
    if first:
        real_ref_x.append(x0)
        first = False
    elif increment:
        real_ref_x.append(real_ref_x[ii - 1] + v * dt)
    else:
        real_ref_x.append(real_ref_x[ii - 1])
# Building theoretical y position:
first = True
start_time_side2_y = end_time_side1_x + delay_side1_x + time_turn + delay_turn    # 9.9
end_time_side2_y = start_time_side2_y + time_side2_y
start_time_side4_y = end_time_side3_x + delay_side3_x + time_turn + delay_turn   # 18.46
end_time_side4_y = start_time_side4_y + time_side4_y

key_times = [start_time_side2_y, end_time_side2_y, start_time_side4_y, end_time_side4_y, 500]
pos = 0
increment = False
v = 0.2
for ii in range(0, len(real_ref_vx)):
    if real_time_ref[ii] >= key_times[pos]:
        pos += 1
        increment = not increment
    if pos == 2:
        v = - 0.2
    if first:
        real_ref_y.append(x0)
        first = False
    elif increment:
        real_ref_y.append(real_ref_y[ii - 1] + v * dt)
    else:
        real_ref_y.append(real_ref_y[ii - 1])

# Building theoretical z position:
first = True

start_time_x = time_initial_takeoff
end_time_z = end_time_side4_y + delay_side4_y

for ii in range(0, len(real_ref_vx)):
    if real_time_ref[ii] <= end_time_z and real_time_ref[ii] > start_time_x:
        real_ref_z.append(0.3)
    else:
        real_ref_z.append(0)

# Building reference wz:
start_time_turn1 = end_time_side1_x + delay_side1_x           # 7.65
end_time_turn1 = start_time_turn1 + time_turn
start_time_turn2 = end_time_side2_y + delay_side2_y           # 11.9
end_time_turn2 = start_time_turn2 + time_turn
start_time_turn3 = end_time_side3_x + delay_side3_x
end_time_turn3 = start_time_turn3 + time_turn

real_ref_wz_theoretical = []
for ii in range(0, len(real_ref_vx)):
    if (real_time_ref[ii] <= end_time_turn1 and real_time_ref[ii] > start_time_turn1) or (real_time_ref[ii] <= end_time_turn2 and real_time_ref[ii] > start_time_turn2) or (real_time_ref[ii] <= end_time_turn3 and real_time_ref[ii] > start_time_turn3):
        real_ref_wz_theoretical.append(72)
    else:
        real_ref_wz_theoretical.append(0)

'''for ii in range(0, len(real_time_state)):
    real_time_state[ii] = real_time_state[ii] - 2.3
    real_time_motor[ii] = real_time_motor[ii] - 2.3
    real_time_ref[ii] = real_time_ref[ii] - 2.3'''

# Figure state position:
if show_real_position:
    fig_cont += 1
    figure(fig_cont)
    # X position:
    subplot(311)
    plot(real_time_state, real_x, label='Real')
    plot(real_time_state, real_ref_x, label='Theor')
    ylabel('X [m]')
    title('Real crazyflie position')
    legend(loc="upper right")
    xlim(0, 25)

    # Y position:
    subplot(312)
    plot(real_time_state, real_y, label='Real')
    plot(real_time_state, real_ref_y, label='Theor')
    ylabel('Y [m]')
    legend(loc="upper right")
    xlim(0, 25)

    # Z position:
    subplot(313)
    plot(real_time_state, real_z, label='Real')
    plot(real_time_state, real_ref_z, label='Theor')
    ylabel('Z [m]')
    xlabel('Time [s]')
    legend(loc="center bottom")
    xlim(0, 25)

# Figure velocity
if show_real_velocity:
    fig_cont += 1
    figure(fig_cont)
    # Real Vx VS actual Vx:
    subplot(411)
    plot(real_time_state, real_vx, label="Real")
    plot(real_time_ref, real_ref_vx, label="Ref")
    ylabel('Vx [m/s]')
    title('Real crazyflie velocity')
    legend(loc="upper right")
    xlim(0, 25)

    # Real Vy VS actual Vy:
    subplot(412)
    plot(real_time_state, real_vy, label="Real")
    plot(real_time_ref, real_ref_vy, label="Ref")
    ylabel('Vy [m/s]')
    legend(loc="upper right")
    xlim(0, 25)

    # Real Vz VS actual Vz:
    subplot(413)
    plot(real_time_state, real_vz, label="Real")
    plot(real_time_ref, real_ref_vz, label="Ref")
    ylabel('Vz [m/s]')
    legend(loc="upper right")
    xlim(0, 25)

    # Real Wz VS actual Wz:
    subplot(414)
    plot(real_time_state, real_wz, label="Real")
    #plot(real_time_ref, real_ref_wz, label="Ref")
    plot(real_time_ref, real_ref_wz_theoretical, label="Ref")
    ylabel('Wz [deg/s]')
    xlabel('Time [s]')
    legend(loc="upper right")
    xlim(0, 25)

# Figure orientation:
if show_real_orientation:
    fig_cont += 1
    figure(fig_cont)
    # Roll value:
    subplot(311)
    plot(real_time_state, real_roll)
    ylabel('R [deg]')
    title('Real crazyflie orientation')
    xlim(0, 25)

    # Pitch value:
    subplot(312)
    plot(real_time_state, real_pitch)
    ylabel('P [deg]')
    xlim(0, 25)

    # Yaw value:
    subplot(313)
    plot(real_time_state, real_yaw)
    ylabel('Y [deg]')
    xlabel('Time [s]')
    xlim(0, 25)

# Figure motor commands:
if show_real_motor_command:
    fig_cont += 1
    figure(fig_cont)
    # Roll command value:
    subplot(411)
    plot(real_time_motor, real_roll_cmd)
    ylabel('R [u]')
    title('Real crazyflie motor commands')
    xlim(0, 25)

    # Pitch command value:
    subplot(412)
    plot(real_time_motor, real_pitch_cmd)
    ylabel('P [u]')
    xlim(0, 25)

    # Yaw command value:
    subplot(413)
    plot(real_time_motor, real_yaw_cmd)
    ylabel('Y [u]')
    xlim(0, 25)

    # Thrust command value:
    subplot(414)
    plot(real_time_motor, real_thrust_cmd)
    ylabel('T [u]')
    xlabel('Time [s]')
    xlim(0, 25)



# ======================================================================================================================
#                               C O M P A R I S O N S  R E A L  V S  S I M U L A T I O N
# ======================================================================================================================
# Position comparison figure:
if show_comp_position:
    fig_cont += 1
    figure(fig_cont)

    new_real_time_state = array(real_time_state) - 2.8
    new_sim_time_state = array(sim_time_state) - 2.8
    new_ref_time = array(real_time_ref) - 2.8
    # X position:
    subplot(211)
    plot(new_real_time_state, real_x, label='Real')
    plot(new_sim_time_state, sim_x, label='Sim')
    plot(new_ref_time, real_ref_x, label='Theor')
    ylabel('X [m]')
    title('Position comparison')
    legend(loc="upper right")
    xlim(0, 20)

    # Y position:
    subplot(212)
    plot(new_real_time_state, real_y, label='Real')
    plot(new_sim_time_state, sim_y, label='Sim')
    plot(new_ref_time, real_ref_y, label='Theor')
    ylabel('Y [m]')
    legend(loc="upper right")
    xlim(0, 20)
    ylim(-0.1, 0.4)
    xlabel('Time [s]')

    # Z position:
    '''subplot(313)
    plot(new_real_time_state, real_z, label='Real')
    plot(new_sim_time_state, sim_z, label='Sim')

    #plot(sim_fake_time_state, sim_fake_z, label='Sim')
    plot(new_ref_time, real_ref_z, label='Theor')
    ylabel('Z [m]')
    xlabel('Time [s]')
    legend(loc="bottom center")
    xlim(0, 20)'''

if show_comp_velocity:
    fig_cont += 1
    figure(fig_cont)

    new_real_time_state = array(real_time_state) - 2.8
    new_sim_time_state = array(sim_time_state) - 2.8
    new_ref_time = array(real_time_ref) - 2.8

    real_vx = array(real_vx)
    sim_vx = array(sim_vx)
    print(real_vx.shape)
    print(sim_vx.shape)
    equiv_real_vx = real_vx[argmax(new_real_time_state>0):argmax(new_real_time_state>20)]
    equiv_sim_vx = sim_vx[0:argmax(new_sim_time_state>20)]
    print('Lunghezza sim:', equiv_real_vx.shape)
    print('Lunghezza real:', equiv_sim_vx.shape)

    # Real Vx VS actual Vx:
    subplot(311)
    plot(new_real_time_state, real_vx, label="Real")
    plot(new_sim_time_state, sim_vx, label='Sim')
    plot(new_ref_time, real_ref_vx, label="Ref")
    ylabel('Vx [m/s]')
    title('Velocity comparison')
    legend(loc="upper right")
    xlim(0, 20)

    # Real Vy VS actual Vy:
    subplot(312)
    plot(new_real_time_state, real_vy, label="Real")
    plot(new_sim_time_state, sim_vy, label='Sim')
    plot(new_ref_time, real_ref_vy, label="Ref")
    ylabel('Vy [m/s]')
    legend(loc="upper right")
    xlim(0, 20)

    # Real Vz VS actual Vz:
    '''subplot(413)
    plot(new_real_time_state, real_vz, label="Real")
    plot(new_sim_time_state, sim_vz, label='Sim')
    plot(new_ref_time, real_ref_vz, label="Ref")
    ylabel('Vz [m/s]')
    legend(loc="upper right")
    ylim(-0.2, 0.7)
    xlim(0, 20)'''

    # Real Wz VS actual Wz:
    subplot(313)
    plot(new_real_time_state, real_wz, label="Real")
    # plot(real_time_ref, real_ref_wz, label="Ref")
    plot(new_sim_time_state, sim_wz, label='Sim')
    plot(new_ref_time, real_ref_wz_theoretical, label="Ref")
    ylabel('Wz [deg/s]')
    xlabel('Time [s]')
    legend(loc="upper right")
    xlim(0, 20)

if show_comp_orientation:
    fig_cont += 1
    figure(fig_cont)
    sim_time_head = linspace(0, 2.8, 45)
    zeros_arr = zeros((len(sim_time_head)))
    new_sim_time_state = array(sim_time_state)
    new_sim_roll = array(sim_roll)
    new_real_time_state = array(real_time_state)
    new_real_time_state = new_real_time_state - 2.8
    new_sim_time_state = new_sim_time_state - 2.8
    # Roll value:
    subplot(311)
    plot(new_real_time_state, real_roll, label='Real')
    plot(new_sim_time_state, new_sim_roll, label='Sim')
    ylabel('R [deg]')
    title('Orientation comparison')
    xlim(0, 20)
    ylim(-4, 4)
    legend(loc="upper right")

    print(len(sim_time_state))
    print(len(real_time_state))
    print(sim_time_state[0])
    print(real_time_state[0])

    # Pitch value:
    subplot(312)
    plot(new_real_time_state, real_pitch, label='Real')
    plot(new_sim_time_state, sim_pitch, label='Sim')
    ylabel('P [deg]')
    xlim(0, 20)
    legend(loc="upper right")

    # Yaw value:
    subplot(313)
    plot(new_real_time_state, real_yaw, label='Real')
    plot(new_sim_time_state, sim_yaw, label='Sim')
    ylabel('Y [deg]')
    xlabel('Time [s]')
    xlim(0, 20)
    legend(loc="upper right")

if show_comp_motor_command:
    fig_cont += 1
    figure(fig_cont)
    # Roll command value:
    subplot(411)
    plot(sim_time_motor, sim_roll_cmd, label='Sim')
    plot(real_time_motor, real_roll_cmd, label='Real')

    ylabel('R [u]')
    xlim(0, 20)
    title('Motor commands comparison')
    legend(loc="upper right")

    # Pitch command value:
    subplot(412)
    plot(sim_time_motor, sim_pitch_cmd, label='Sim')
    plot(real_time_motor, real_pitch_cmd, label='Real')

    ylabel('P [u]')
    xlim(0, 20)
    legend(loc="upper right")

    # Yaw command value:
    subplot(413)
    plot(sim_time_motor, sim_yaw_cmd, label='Sim')
    plot(real_time_motor, real_yaw_cmd, label='Real')

    ylabel('Y [u]')
    xlim(0, 20)
    legend(loc="upper right")

    # Thrust command value:
    subplot(414)
    plot(sim_time_motor, sim_thrust_cmd, label='Sim')
    plot(real_time_motor, real_thrust_cmd, label='Real')

    ylabel('T [u]')
    xlabel('Time [s]')
    xlim(0, 20)
    legend(loc="upper right")

print(len(real_roll), len(sim_roll))
show()