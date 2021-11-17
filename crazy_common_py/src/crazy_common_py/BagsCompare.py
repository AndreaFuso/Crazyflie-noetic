import rosbag
import rospkg
from crazy_common_py.common_functions import rad2deg

from matplotlib.pyplot import plot, xlabel, ylabel, show, figure, title, ylim, subplot, xlim, legend
from numpy import linspace

rospack = rospkg.RosPack()

bag_name_real = 'real_state.bag'
bag_name_sim = 'sim_state_2.bag'
bag_name_sim_motor = 'sim_motor_2.bag'
bag_name_sim_ref = 'sim_ref_2.bag'

bag_path_real = rospack.get_path('crazyCmd') + '/data/output/Rosbags/' + bag_name_real
bag_path_sim = rospack.get_path('crazyCmd') + '/data/output/Rosbags/' + bag_name_sim
bag_path_sim_motor = rospack.get_path('crazyCmd') + '/data/output/Rosbags/' + bag_name_sim_motor
bag_path_sim_ref = rospack.get_path('crazyCmd') + '/data/output/Rosbags/' + bag_name_sim_ref


bag_real = rosbag.Bag(bag_path_real)
bag_sim = rosbag.Bag(bag_path_sim)
bag_sim_motor = rosbag.Bag(bag_path_sim_motor)
bag_sim_ref = rosbag.Bag(bag_path_sim_ref)

# Real angles
roll_real = []
pitch_real = []
yaw_real = []
time_sec_real = []
time_nsec_real = []

# Sim angles:
roll_sim = []
pitch_sim = []
yaw_sim = []
time_sec_sim = []
time_nsec_sim = []

# Real linear vel:
vx_real = []

# Sim linear vel:
vx_sim = []
vy_sim = []
vz_sim = []
vx_sim_ref = []
vy_sim_ref = []
vz_sim_ref = []
# Sim rot speed:
yaw_rate_sim = []
yaw_rate_sim_ref = []

# Real motor commands:
roll_cmd_real = []
pitch_cmd_real = []
yaw_cmd_real = []
thrust_cmd_real = []

# Sim motor commands:
roll_cmd_sim = []
pitch_cmd_sim = []
yaw_cmd_sim = []
thrust_cmd_sim = []

# Filling real state:
first = True
initial_time_nsec = 0
for topic, msg, t in bag_real.read_messages(topics=['/cf1/state']):
    roll_real.append(rad2deg(msg.orientation.roll))
    pitch_real.append(rad2deg(msg.orientation.pitch))
    yaw_real.append(rad2deg(msg.orientation.yaw))


    time_sec_real.append(t.secs)
    if first:
        initial_time_nsec = t.nsecs

        time_nsec_real.append(0.0)
        first = False
    else:
        time_nsec_real.append((t.nsecs - initial_time_nsec) / (10**9))

# Filling real motor command:

bag_real.close()

# Filling sim:
for topic, msg, t in bag_sim.read_messages(topics=['/cf1/state']):

    roll_sim.append(rad2deg(msg.orientation.roll))
    pitch_sim.append(rad2deg(msg.orientation.pitch))
    yaw_sim.append(rad2deg(msg.orientation.yaw))

    vx_sim.append(msg.velocity.x)
    vy_sim.append(msg.velocity.y)
    vz_sim.append(msg.velocity.z)

    yaw_rate_sim.append(rad2deg(msg.rotating_speed.z))

    time_sec_sim.append(t.secs)
    time_nsec_sim.append(t.nsecs)


bag_sim.close()


for topic, msg, t in bag_sim_ref.read_messages(topics=['/cf1/actual_state_target']):

    vx_sim_ref.append(msg.desired_velocity.x)
    vy_sim_ref.append(msg.desired_velocity.y)
    vz_sim_ref.append(msg.desired_velocity.z)

    yaw_rate_sim_ref.append(msg.desired_yaw)

bag_sim_ref.close()

for topic, msg, t in bag_sim_motor.read_messages(topics=['/cf1/motor_command']):

    roll_cmd_sim.append(msg.desired_attitude.roll)
    pitch_cmd_sim.append(msg.desired_attitude.pitch)
    yaw_cmd_sim.append(msg.desired_attitude.yaw)
    thrust_cmd_sim.append(msg.desired_thrust)

bag_sim_motor.close()



# Plotting
'''time_real = linspace(0.0, 29.4, num=len(roll_real))
time_sim = linspace(0.0, 27.5, num=len(roll_sim))

print(list(time_real).index(10))
print(time_sim[209])

time_real_part = time_real[1000:]
time_real_part = time_real_part - time_real_part[0]

sim_cut = 765
time_sim_part = time_sim[sim_cut:]
time_sim_part = time_sim_part - time_sim_part[0]

figure(1)
subplot(211)
plot(time_real_part, pitch_real[1000:])
ylabel('Pitch angle [deg]')
title('Real')
ylim(-5, 5)
xlim(0, 17.5)
subplot(212)
plot(time_real_part, yaw_real[1000:])
xlabel('Time [s]')
ylabel('Yaw angle [deg]')
xlim(0, 17.5)



figure(2)
subplot(211)
plot(time_sim_part, pitch_sim[sim_cut:])
title('Simulation')
ylabel('Pitch angle [deg]')
ylim(-5, 5)
xlim(0, 17.5)
subplot(212)
plot(time_sim_part, yaw_sim[sim_cut:])
xlabel('Time [s]')
ylabel('Yaw angle [deg]')
xlim(0, 17.5)

figure(3)
subplot(211)
plot(time_real_part, pitch_real[1000:], label='Real')
plot(time_sim_part, pitch_sim[sim_cut:], label='Simulated')
title('Real VS Simulation')
ylabel('Pitch angle [deg]')
legend(loc="upper right")
ylim(-5, 5)
xlim(0, 17.5)
subplot(212)
plot(time_real_part, yaw_real[1000:], label='Real')
plot(time_sim_part, yaw_sim[sim_cut:], label='Simulated')
xlabel('Time [s]')
ylabel('Yaw angle [deg]')
legend(loc="upper right")
xlim(0, 17.5)'''

figure(4)
time_state_sim = linspace(0.0, 30.9, num=len(roll_sim))
time_motor_sim = linspace(0.0, 30.9, num=len(roll_cmd_sim))
time_motor_sim_ref = linspace(0.0, 21.2, num=len(vx_sim_ref))

subplot(811)
plot(time_motor_sim, roll_cmd_sim)
title('SIM')
ylabel('Rcmd')

subplot(812)
plot(time_motor_sim, pitch_cmd_sim)
ylabel('Pcmd')

subplot(813)
plot(time_motor_sim, yaw_cmd_sim)
ylabel('Ycmd')

subplot(814)
plot(time_motor_sim, thrust_cmd_sim)
ylabel('Tcmd')

subplot(815)
plot(time_state_sim, vx_sim, label='Actual')
plot(time_motor_sim_ref, vx_sim_ref, label='Ref')
legend(loc="upper right")
ylabel('Vx')

subplot(816)
plot(time_state_sim, vy_sim, label='Actual')
plot(time_motor_sim_ref, vy_sim_ref, label='Ref')
legend(loc="upper right")
ylabel('Vy')

subplot(817)
plot(time_state_sim, vz_sim, label='Actual')
plot(time_motor_sim_ref, vy_sim_ref, label='Ref')
legend(loc="upper right")
ylabel('Vz')

subplot(818)
plot(time_state_sim, yaw_rate_sim, label='Actual')
plot(time_motor_sim_ref, yaw_rate_sim_ref, label='Ref')
legend(loc="upper right")
ylabel('Yrate')
xlabel('Time [s]')
show()
