import rosbag
import rospkg
from crazy_common_py.common_functions import rad2deg
from crazy_common_py.default_topics import DEFAULT_CF_STATE_TOPIC, DEFAULT_MOTOR_CMD_TOPIC, \
    DEFAULT_ACTUAL_DESTINATION_TOPIC
from matplotlib.pyplot import plot, xlabel, ylabel, show, figure, title, ylim, subplot, xlim, legend
from rosgraph_msgs.msg import Clock
from numpy import linspace, full, array, asarray
from scipy.signal import convolve2d
from librosa.core import resample
from librosa.effects import time_stretch

rospack = rospkg.RosPack()

# ======================================================================================================================
#                                                   S E T T I N G S
# ======================================================================================================================
time_round_precision = 1
fig_cont = 0
isSimTime = True

# Real bag:
real_state_bag_name = 'real_state_S_D_02_02_02_02_V_02_02_02_02_RA_90_RR_72_N4.bag'
real_motor_bag_name = 'real_motor_S_D_02_02_02_02_V_02_02_02_02_RA_90_RR_72_N4.bag'
real_ref_bag_name = 'real_ref_S_D_02_02_02_02_V_02_02_02_02_RA_90_RR_72_N4.bag'
# Simulation bag:
sim_state_bag_name = 'sim_state_S_D_02_02_02_02_V_02_02_02_02_RA_90_RR_72_N5.bag'
sim_motor_bag_name = 'sim_motor_S_D_02_02_02_02_V_02_02_02_02_RA_90_RR_72_N5.bag'
sim_ref_bag_name = 'sim_ref_S_D_02_02_02_02_V_02_02_02_02_RA_90_RR_72_N5.bag'
if isSimTime:
    sim_clock_bag_name = 'sim_clock_S_D_02_02_02_02_V_02_02_02_02_RA_90_RR_72_N5.bag'

# ======================================================================================================================
#                                   S I M U L A T E D  D A T A  E X T R A C T I O N
# ======================================================================================================================
# Rosbags path definitions:
sim_state_bag_path = rospack.get_path('crazyCmd') + '/data/output/Rosbags/' + sim_state_bag_name
sim_motor_bag_path = rospack.get_path('crazyCmd') + '/data/output/Rosbags/' + sim_motor_bag_name
sim_ref_bag_path = rospack.get_path('crazyCmd') + '/data/output/Rosbags/' + sim_ref_bag_name
if isSimTime:
    sim_clock_bag_path = rospack.get_path('crazyCmd') + '/data/output/Rosbags/' + sim_clock_bag_name

# Rosbags instances:
sim_state_bag = rosbag.Bag(sim_state_bag_path)
sim_motor_bag = rosbag.Bag(sim_motor_bag_path)
sim_ref_bag = rosbag.Bag(sim_ref_bag_path)
if isSimTime:
    sim_clock_bag = rosbag.Bag(sim_clock_bag_path)

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

sim_time_state = []
sim_time_cmd = []
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

    sim_time_state.append(t.to_sec())

sim_state_secs = round(sim_state_bag.get_end_time() - sim_state_bag.get_start_time(), time_round_precision)
sim_state_bag.close()

# Extracting data about motor commands:
for topic, msg, t in sim_motor_bag.read_messages(topics=['/cf1/'+DEFAULT_MOTOR_CMD_TOPIC]):
    # Sim motor commands:
    sim_roll_cmd.append(msg.desired_attitude.roll)
    sim_pitch_cmd.append(msg.desired_attitude.pitch)
    sim_yaw_cmd.append(msg.desired_attitude.yaw)
    sim_thrust_cmd.append(msg.desired_thrust)

    sim_time_cmd.append(t.to_sec())
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

# Clock from simulation:
if isSimTime:
    sim_time = []
    for topic, msg, t in sim_clock_bag.read_messages(topics=['/clock']):
        sim_time.append(msg.clock.to_sec())
    sim_clock_bag.close()
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

real_time_state = []
real_time_cmd = []
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

    real_time_state.append(t.to_sec())

real_state_secs = round(real_state_bag.get_end_time() - real_state_bag.get_start_time(), time_round_precision)
real_state_bag.close()

# Extracting real motor command:
for topic, msg, t in real_motor_bag.read_messages(topics=['/cf1/'+DEFAULT_MOTOR_CMD_TOPIC]):
    # Real motor commands:
    real_roll_cmd.append(msg.desired_attitude.roll)
    real_pitch_cmd.append(msg.desired_attitude.pitch)
    real_yaw_cmd.append(msg.desired_attitude.yaw)
    real_thrust_cmd.append(msg.desired_thrust)

    real_time_cmd.append(t.to_sec())

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

# ======================================================================================================================
#                                     R E F E R E N C E  C R E A T I O N
# ======================================================================================================================

# ======================================================================================================================
#                                               F I G U R E S
# ======================================================================================================================
# ----------------------------------------------------------------------------------------------------------------------
#                                           P O S I T I O N  C O M P A R I S O N
# ----------------------------------------------------------------------------------------------------------------------
sim_time_state = linspace(0, sim_state_secs, len(sim_x))
real_time_state = linspace(0, real_state_secs, len(real_x))


sim_time_state = array(sim_time_state)
real_time_state = array(real_time_state)

sim_time_state = sim_time_state - sim_time_state[0]
real_time_state = real_time_state - real_time_state[0]

#result = convolve2d(in_array.astype(int), kernel.astype(int), mode='same').astype(bool)


real_pitch_resemp = resample(array(real_pitch), len(real_pitch), len(sim_pitch))
'''rate_stretch = (len(real_pitch)/len(sim_pitch)) * 1
real_pitch_resemp = time_stretch(array(real_pitch), rate=1.136)'''
real_state_time_resemp = linspace(0, sim_state_secs, len(real_pitch_resemp))
print(real_pitch_resemp.shape)
#real_state_time_resemp = real_state_time_resemp - 1.04

sim_pitch_resemp = resample(array(sim_pitch), len(sim_pitch), len(real_pitch), fix=False, scale=True)
print(len(sim_pitch))
print(sim_pitch_resemp.shape)

'''rate_stretch = (len(real_pitch)/len(sim_pitch)) * 1
sim_pitch_resemp = time_stretch(array(real_pitch), rate=0.88)'''
sim_state_time_resemp = linspace(0, real_state_secs, len(real_time_state))
sim_state_time_resemp = time_stretch(sim_time_state, rate=0.88)
sim_pitch_resemp = resample(array(sim_pitch), len(sim_pitch), len(sim_state_time_resemp), fix=False, scale=True)


sim_time_cmd = array(sim_time_cmd)
real_time_cmd = array(real_time_cmd)

sim_time_cmd = sim_time_cmd - sim_time_cmd[0]
real_time_cmd = real_time_cmd - real_time_cmd[0]

fig_cont = 1
figure(fig_cont)
plot(sim_time_state, sim_x, label='Sim')
plot(real_time_state, real_x, label='Real')
ylabel('X [m]')
xlabel('Time [s]')

title('Position comparison')
legend(loc='upper right')

# ----------------------------------------------------------------------------------------------------------------------
#                                          R O L L  P I T C H  Y A W
# ----------------------------------------------------------------------------------------------------------------------
fig_cont += 1
figure(fig_cont)
subplot(311)
plot(sim_time_state, sim_roll, label='Sim')
plot(real_time_state, real_roll, label='Real')
ylabel('R [deg]')
legend(loc='upper right')
title('Roll Pitch Yaw comparison')
subplot(312)
plot(sim_time_state, sim_pitch, label='Sim')
plot(real_time_state, real_pitch, label='Real')
#plot(real_state_time_resemp, real_pitch_resemp, label='Real res')
plot(sim_state_time_resemp, sim_pitch_resemp, label='Sim res')
ylabel('P [deg]')
legend(loc='upper right')
subplot(313)
plot(sim_time_state, sim_yaw, label='Sim')
plot(real_time_state, real_yaw, label='Real')
ylabel('Y [deg]')
xlabel('Time [s]')
legend(loc='upper right')

# ----------------------------------------------------------------------------------------------------------------------
#                                          C O M M A N D S
# ----------------------------------------------------------------------------------------------------------------------
fig_cont += 1
figure(fig_cont)
subplot(411)
plot(sim_time_cmd, sim_roll_cmd, label='Sim')
plot(real_time_cmd, real_roll_cmd, label='Real')
ylabel('R [u]')
legend(loc='upper right')
title('Commands comparison')
subplot(412)
plot(sim_time_cmd, sim_pitch_cmd, label='Sim')
plot(real_time_cmd, real_pitch_cmd, label='Real')
ylabel('P [u]')
legend(loc='upper right')
subplot(413)
plot(sim_time_cmd, sim_yaw_cmd, label='Sim')
plot(real_time_cmd, real_yaw_cmd, label='Real')
ylabel('Y [u]')
legend(loc='upper right')
subplot(414)
plot(sim_time_cmd, sim_thrust_cmd, label='Sim')
plot(real_time_cmd, real_thrust_cmd, label='Real')
ylabel('T [u]')
xlabel('Time [s]')
legend(loc='upper right')

show()
