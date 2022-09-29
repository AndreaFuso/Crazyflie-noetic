#! /usr/bin/env python3

# in this script, we will write python codes to do the basic complementary filter, 
# our target is to get the estimated roll, pitch, yaw

# ROS MODULES
import rospy

# Other modules
import math

# MESSAGES
from sensor_msgs.msg import Imu
from crazyflie_messages.msg import RollPitchYaw

class CompleFilter():
    def __init__(self):
        # list some parameters
        self.updateRate =  100 #Hz
        self.dt = 1/self.updateRate #s
        self.hpf = 0.98
        self.lpf = 0.02

        self.rpy_filter = RollPitchYaw()

        self.rate = rospy.Rate(100)

    # initialize rpy, angular velocities and linear acceleration
        self.thetaX, self.thetaY, self.thetaZ = 0, 0, 0
        self.gyr_x, self.gyr_y, self.gyr_z = 0, 0, 0
        self.acc_x, self.acc_y, self.acc_z = 0, 0, 0
        self.thetaX_gyr, self.thetaY_gyr, self.thetaZ_gyr = 0, 0, 0
        self.thetaX_acc, self.thetaY_acc, self.thetaZ_acc = 0, 0, 0

        rospy.Subscriber('/cf1/imu',Imu, self.clbk_imu)
        self.comple_filter_pub = rospy.Publisher('/cf1/rpy_comple_filter',RollPitchYaw, queue_size=1)

    def clbk_imu(self,msg):
        # get accleration form msg
        self.acc_x = msg.linear_acceleration.x
        self.acc_y = msg.linear_acceleration.y
        self.acc_z = msg.linear_acceleration.z

        # get angular velocity from msg
        self.gyr_x = msg.angular_velocity.x
        self.gyr_y = msg.angular_velocity.y
        self.gyr_z = msg.angular_velocity.z

    # def angles_from_acc(self):
        # compute angles from acclerometers
        self.thetaX_acc = math.atan2(self.acc_y,math.sqrt(self.acc_x**2+self.acc_z**2))
        self.thetaY_acc = math.atan2(self.acc_x,math.sqrt(self.acc_y**2+self.acc_z**2))
        # self.thetaZ_acc = math.atan2(self.acc_y,math.sqrt(self.acc_x**2+self.acc_z**2))
        self.thetaZ_acc = 0

    # def angles_from_gyr(self):
         # compute angles from gyroscopes
        self.thetaX_gyr = self.thetaX_gyr + self.gyr_x*self.dt
        self.thetaY_gyr = self.thetaY_gyr + self.gyr_y*self.dt
        self.thetaZ_gyr = self.thetaZ_gyr + self.gyr_z*self.dt     

    def take_action_publish_angles(self):
        while not rospy.is_shutdown():
            # if self.thetaX==0 and self.thetaY==0 and self.thetaZ==0:
            #     continue

            # else:
                # combine two angles
            self.thetaX = (self.hpf*self.thetaX_gyr+ self.lpf*self.thetaX_acc)*180/math.pi
            self.thetaY = (self.hpf*self.thetaY_gyr + self.lpf*self.thetaY_acc)*180/math.pi
            self.thetaZ = (self.hpf*self.thetaZ_gyr + self.lpf*self.thetaZ_acc)*180/math.pi

            # introduce into RollPitchYaw message
            self.rpy_filter.roll = self.thetaX
            self.rpy_filter.pitch = self.thetaY
            self.rpy_filter.yaw = self.thetaZ
            self.comple_filter_pub.publish(self.rpy_filter)

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node("imu_comple_filter_hand")
    my_node = CompleFilter()
    my_node.take_action_publish_angles()
