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
frequency = 1000
N_state = 23

class pipeline:
    def __init__(self):

        self.pub1 = rospy.Publisher('/snake2/joint1_position_controller/command', Float64, queue_size=100)
        self.pub2 = rospy.Publisher('/snake2/joint2_position_controller/command', Float64, queue_size=100)
        self.pub3 = rospy.Publisher('/snake2/joint3_position_controller/command', Float64, queue_size=100)
        self.pub4 = rospy.Publisher('/snake2/joint4_position_controller/command', Float64, queue_size=100)
        self.pub5 = rospy.Publisher('/snake2/joint5_position_controller/command', Float64, queue_size=100)
        self.pub6 = rospy.Publisher('/snake2/joint6_position_controller/command', Float64, queue_size=100)
        self.pub7 = rospy.Publisher('/snake2/joint7_position_controller/command', Float64, queue_size=100)
        self.pub8 = rospy.Publisher('/snake2/joint8_position_controller/command', Float64, queue_size=100)
        self.pub9 = rospy.Publisher('/snake2/joint9_position_controller/command', Float64, queue_size=100)
        self.pub10 = rospy.Publisher('/snake2/joint10_position_controller/command', Float64, queue_size=100)
        self.pub11 = rospy.Publisher('/snake2/joint11_position_controller/command', Float64, queue_size=100)
        self.pub12 = rospy.Publisher('/snake2/joint12_position_controller/command', Float64, queue_size=100)
        self.pub13 = rospy.Publisher('/snake2/joint13_position_controller/command', Float64, queue_size=100)
        self.pub14 = rospy.Publisher('/snake2/joint14_position_controller/command', Float64, queue_size=100)
        self.pub15 = rospy.Publisher('/snake2/joint15_position_controller/command', Float64, queue_size=100)
        self.pub16 = rospy.Publisher('/snake2/joint16_position_controller/command', Float64, queue_size=100)

        self.pub17 = rospy.Publisher('/snake/thetaError', Float64, queue_size=100)

        # rospy.init_node('pipeline', anonymous=True)
        self.rate = rospy.Rate(frequency)  # publishing frequency

        self.gazebo_state = Gazebo_data()

        self.joint_angle = np.zeros((N, 1), dtype=np.float64) # joint angle / rad
        self.max_joint_angle = np.pi / 2.0 # maximun joint angle

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

        self.rate.sleep() # sleep for a while to keep publishing frequency 



    def step(self, action, dt):
        for i in range (N):
            self.joint_angle[i] = action[i]
        t1 = time.time()
        while (time.time() - t1 <= dt):
           self.joint_angle_set()
        
        # self.joint_angle_set()    
        state = np.zeros(N_state, dtype=np.float64)
        for i in range(N):
            state[i] = self.gazebo_state.joint_position[i, 0]
        for i in range(3):
            state[i + 16] = self.gazebo_state.head_position[i, 0]
        for i in range(4):
            state[i + 19] = self.gazebo_state.head_orientation[i, 0]

    def read_joint_angle(self):
        return Gazebo_data.joint_position

    def read_joint_effort(self):
        return Gazebo_data.joint_effort

       

        #r = self.gazebo_state.head_position[0, 0]-0.693-abs(self.gazebo_state.head_position[1, 0]+0.107)  # height of the head
        # r = self.gazebo_state.head_position[0, 0]-0.693  # height of the head
        #r = self.gazebo_state.head_position[2, 0]
        # print state
        
        # return state,r

# if __name__ == '__main__':
#     sw =pipeline()
#     action = np.ones(N, dtype=np.float64) * 0.0
#     t1 = time.time()
#     while (time.time() - t1 <= 0.05):
#         sw.step(action)
#     t2 = time.time()
#     print (t2-t1)

