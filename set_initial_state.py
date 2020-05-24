#!/usr/bin/env python
'''pytest ROS Node'''
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelState
import numpy as np
import time

from robot_co_environment import Gazebo_data

N = 16
frequency = 50
N_state = 23

class reset_state:
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

        self.pub_model_state = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        self.pose_msg = ModelState()
        self.pose_msg.model_name = 'snake2'



        # rospy.init_node('set_initial_state', anonymous=True)
        self.rate = rospy.Rate(frequency)  # publishing frequency

        self.gazebo_state = Gazebo_data()

        self.joint_angle = np.zeros((N, 1), dtype=np.float64) # joint angle / rad
        self.max_joint_angle = np.pi / 2.0 # maximun joint angle

        self.A = 4 * np.pi / 9
        self.phi = np.pi / 6
        self.v = np.pi / 8
        self.current_time = 0.0

    def model_state_set(self):
        self.pose_msg.pose.position.x = 0.693
        # self.pose_msg.pose.position.y = -0.12
        self.pose_msg.pose.position.y = -0.120
        self.pose_msg.pose.position.z = 0.064

        self.pose_msg.pose.orientation.x = -0.049
        self.pose_msg.pose.orientation.y = 0.0
        self.pose_msg.pose.orientation.z = -0.007
        self.pose_msg.pose.orientation.w = 0.999

        self.pub_model_state.publish(self.pose_msg)
        self.rate.sleep()

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

    def set_initial_state(self):
        for i in range(N):
            self.joint_angle[i] = 0
        # print(self.joint_angle)
        # self.joint_angle[0] = np.pi / 2
        t1 = time.time()
        while (time.time() - t1 <= 2.5):
            self.joint_angle_set()
        t2 = time.time()
        while (time.time() - t2 <= 0.05):
            self.model_state_set()
        state = np.zeros(N_state, dtype=np.float64)
        for i in range(N):
            state[i] = self.gazebo_state.joint_position[i, 0]
        for i in range(3):
            state[i + 16] = self.gazebo_state.head_position[i, 0]
        for i in range(4):
            state[i + 19] = self.gazebo_state.head_orientation[i, 0]
        # print state
        # return state



# if __name__ == '__main__':
#     sw =reset_state()
#
#     sw.set_initial_state()
