#!/usr/bin/env python
'''pytest ROS Node'''
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Float64
import numpy as np
import time

from robot_co_environment import Gazebo_data

N = 16
frequency = 50

class sidewinding:
    def __init__(self):

        self.pub1 = rospy.Publisher('/snake2/joint1_position_controller/command', Float64, queue_size=10)
        self.pub2 = rospy.Publisher('/snake2/joint2_position_controller/command', Float64, queue_size=10)
        self.pub3 = rospy.Publisher('/snake2/joint3_position_controller/command', Float64, queue_size=10)
        self.pub4 = rospy.Publisher('/snake2/joint4_position_controller/command', Float64, queue_size=10)
        self.pub5 = rospy.Publisher('/snake2/joint5_position_controller/command', Float64, queue_size=10)
        self.pub6 = rospy.Publisher('/snake2/joint6_position_controller/command', Float64, queue_size=10)
        self.pub7 = rospy.Publisher('/snake2/joint7_position_controller/command', Float64, queue_size=10)
        self.pub8 = rospy.Publisher('/snake2/joint8_position_controller/command', Float64, queue_size=10)
        self.pub9 = rospy.Publisher('/snake2/joint9_position_controller/command', Float64, queue_size=10)
        self.pub10 = rospy.Publisher('/snake2/joint10_position_controller/command', Float64, queue_size=10)
        self.pub11 = rospy.Publisher('/snake2/joint11_position_controller/command', Float64, queue_size=10)
        self.pub12 = rospy.Publisher('/snake2/joint12_position_controller/command', Float64, queue_size=10)
        self.pub13 = rospy.Publisher('/snake2/joint13_position_controller/command', Float64, queue_size=10)
        self.pub14 = rospy.Publisher('/snake2/joint14_position_controller/command', Float64, queue_size=10)
        self.pub15 = rospy.Publisher('/snake2/joint15_position_controller/command', Float64, queue_size=10)
        self.pub16 = rospy.Publisher('/snake2/joint16_position_controller/command', Float64, queue_size=10)

        self.pub17 = rospy.Publisher('/snake/thetaError', Float64, queue_size=10)

        rospy.init_node('sidewinding', anonymous=True)
        self.rate = rospy.Rate(frequency)  # publishing frequency

        self.TR = Gazebo_data()
        time.sleep(0.1)

        self.joint_angle = np.zeros((N, 1), dtype=np.float64) # joint angle / rad
        self.max_joint_angle = np.pi / 2.0 # maximun joint angle

        # parameters of snake curve
        self.K_n = 2  # the number of wave
        self.omega = 1.0 * np.pi  # angular frequency
        self.gamma_yaw = 0.0  # positive right clockwise
        self.gamma_pitch = 0.0  # positive left anticlockwise
        self.delta_phi = np.pi / 12.0  # phase difference between yaw wave and pitch wave
        self.n_yaw = 8
        self.n_pitch = 8  # the number of joint
        self.alpha_yaw = 3.0 / 16.0 * np.pi
        self.alpha_pitch = 2.0 / 16.0 * np.pi  # the amplitute angle
        self.current_time = 0.0

    def joint_angle_set(self):
        for i in range(N):
            if self.joint_angle[i] > self.max_joint_angle:
                self.joint_angle[i] = self.max_joint_angle
            if self.joint_angle[i] < -self.max_joint_angle:
                self.joint_angle[i] = -self.max_joint_angle
        self.pub1.publish(self.joint_angle[0])
        self.pub2.publish(self.joint_angle[1])
        self.pub3.publish(self.joint_angle[2])
        self.pub4.publish(self.joint_angle[3])
        self.pub5.publish(self.joint_angle[4])
        self.pub6.publish(self.joint_angle[5])
        self.pub7.publish(self.joint_angle[6])
        self.pub8.publish(self.joint_angle[7])
        self.pub9.publish(self.joint_angle[8])
        self.pub10.publish(self.joint_angle[9])
        self.pub11.publish(self.joint_angle[10])
        self.pub12.publish(self.joint_angle[11])
        self.pub13.publish(self.joint_angle[12])
        self.pub14.publish(self.joint_angle[13])
        self.pub15.publish(self.joint_angle[14])
        self.pub16.publish(self.joint_angle[15])

        self.rate.sleep() # sleep for a while to make publishing frequency keep 50Hz

    def cal_joint_angles(self):
        for i in range(N / 2):  # sidewinding

            A_yaw = np.sin(self.K_n * np.pi / self.n_yaw)
            angle_rad = -2 * self.alpha_yaw * A_yaw * np.sin(self.omega * self.current_time + i * 2 * self.K_n
                                                             * np.pi / self.n_yaw) + self.gamma_yaw
            self.joint_angle[2 * i] = angle_rad
            A_pitch = np.sin(self.K_n * np.pi / self.n_pitch)
            angle_rad = -2 * self.alpha_pitch * A_pitch * np.sin(self.omega * self.current_time + i * 2 * self.K_n
                                                                 * np.pi / self.n_pitch + self.delta_phi) + self.gamma_pitch
            self.joint_angle[2 * i + 1] = angle_rad
        print(self.joint_angle[0])
        self.joint_angle_set()
        self.current_time = self.current_time + 1.0 / frequency



if __name__ == '__main__':
    sw =sidewinding()
    for i in range(2500):
        sw.cal_joint_angles()
