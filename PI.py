# -*- coding: UTF-8 -*-

import vrep
import numpy
import sys, time, threading
from math import *

roll_outs = 30  # 每一个时间步随机产生的轨迹数
training_times = 60  # 训练次数
N = 8


class PI2:
    def __init__(self):

        self.start = 0, 0  # 起始点位置
        self.goal = 1.5, 1.5  # 目标点位置，米

        self.L = sqrt(pow(self.goal[0] - self.start[0], 2) + pow(self.goal[1] - self.start[1], 2))  # 距离

        self.motion_area_point1 = -0.5, -0.5
        self.motion_area_point2 = 2.5, 2.5  # 运动区域
        self.max_cost = sqrt(pow(self.motion_area_point1[0] - self.motion_area_point2[0], 2) + pow(
            self.motion_area_point1[1] - self.motion_area_point2[1], 2))

        self.constant1 = 30  # 常数1
        self.T = 12  # 运行时间/s
        self.T1 = 6.0  # 中间点时间
        self.dt = 0.05  # 控制周期/s
        self.current_time = 0  # 当前时刻
        self.current_roll_out = 0  # 当前轨迹号
        self.current_train = 0  # 当前训练次数

        self.end_time_position = []  # 终止时刻位置，三维数据,x,y,z
        self.init_obstacle1_position = 0.25, 0.7, 0.05
        self.init_obstacle2_position = 0.95, 0.4, 0.05
        self.obstacle1_position = []
        self.obstacle2_position = []

        # 弧度制
        self.joint_angle = numpy.zeros((N, 1), dtype=numpy.float64)  # 关节角度

        # snake_A = 0.592
        # snake_omega = 6.361
        # snake_beta = 1.046
        # snake_gamma = -0.064

        # 蛇形曲线参数
        self.A1 = 0.592
        self.omega1 = 2 * pi  # [0.5pi,4pi]
        self.beta1 = 1.046
        self.gamma1 = 0  # [-0.12,0.12]

        self.A2 = 0.592
        self.omega2 = 2 * pi  # [0.5pi,4pi]
        self.beta2 = 1.046
        self.gamma2 = 0  # [-0.12,0.12]

        self.A01 = 0.592
        self.omega01 = 2 * pi  # [0.5pi,4pi]
        self.beta01 = 1.046
        self.gamma01 = 0.06  # [-0.12,0.12]

        self.A02 = 0.592
        self.omega02 = 2 * pi  # [0.5pi,4pi]
        self.beta02 = 1.046
        self.gamma02 = 0.06  # [-0.12,0.12]

        # 正态分布标准差
        self.omega_gamma1 = 0.5
        self.gamma_gamma1 = 0.02

        self.omega_gamma2 = 0.5
        self.gamma_gamma2 = 0.02

        # 每条轨迹的训练参数变化量
        self.omega_delta1 = numpy.zeros(roll_outs, dtype=numpy.float64)
        self.gamma_delta1 = numpy.zeros(roll_outs, dtype=numpy.float64)

        self.omega_delta2 = numpy.zeros(roll_outs, dtype=numpy.float64)
        self.gamma_delta2 = numpy.zeros(roll_outs, dtype=numpy.float64)

        # 每条轨迹的训练参数
        self.omega_roll1 = self.omega2 * numpy.ones(roll_outs, dtype=numpy.float64)
        self.gamma_roll1 = self.gamma2 * numpy.ones(roll_outs, dtype=numpy.float64)

        self.omega_roll2 = self.omega2 * numpy.ones(roll_outs, dtype=numpy.float64)
        self.gamma_roll2 = self.gamma2 * numpy.ones(roll_outs, dtype=numpy.float64)

        # 所有的训练参数
        self.omega_all1 = numpy.zeros((training_times, roll_outs), dtype=numpy.float64)
        self.gamma_all1 = numpy.zeros((training_times, roll_outs), dtype=numpy.float64)

        self.omega_all2 = numpy.zeros((training_times, roll_outs), dtype=numpy.float64)
        self.gamma_all2 = numpy.zeros((training_times, roll_outs), dtype=numpy.float64)

        # 每次训练后的损失函数
        self.cost_train = numpy.zeros(training_times, dtype=numpy.float64)
        self.cost_all = numpy.zeros((training_times, roll_outs), dtype=numpy.float64)

        self.cost = numpy.zeros(roll_outs, dtype=numpy.float64)

        # 每次训练后通过PI2计算的参数
        self.omega_train1 = self.omega2 * numpy.ones(training_times, dtype=numpy.float64)
        self.gamma_train1 = self.gamma2 * numpy.ones(training_times, dtype=numpy.float64)

        self.omega_train2 = self.omega2 * numpy.ones(training_times, dtype=numpy.float64)
        self.gamma_train2 = self.gamma2 * numpy.ones(training_times, dtype=numpy.float64)

        #每次训练通过PI2计算的参数，继续产生五条轨迹，取最小损失函数，作为每次迭代后的损失函数
        self.PI2_cost = numpy.zeros(5, dtype=numpy.float64)
        self.PI2_counts = 0


    def sim_start(self):
        print('开始仿真')
        vrep.simxFinish(-1)  # 关闭所有可能的连接
        self.clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # 创建和vrep之间的通信线程
        vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_blocking)  # 开始仿真

        res, self.HandleObstacle1 = vrep.simxGetObjectHandle(self.clientID, "obstacle1", vrep.simx_opmode_oneshot_wait)
        res, self.HandleObstacle2 = vrep.simxGetObjectHandle(self.clientID, "obstacle2", vrep.simx_opmode_oneshot_wait)

        res, self.HandleJoint1 = vrep.simxGetObjectHandle(self.clientID, "yaw_joint0", vrep.simx_opmode_oneshot_wait)
        res, self.HandleJoint2 = vrep.simxGetObjectHandle(self.clientID, "yaw_joint1", vrep.simx_opmode_oneshot_wait)
        res, self.HandleJoint3 = vrep.simxGetObjectHandle(self.clientID, "yaw_joint2", vrep.simx_opmode_oneshot_wait)
        res, self.HandleJoint4 = vrep.simxGetObjectHandle(self.clientID, "yaw_joint3", vrep.simx_opmode_oneshot_wait)
        res, self.HandleJoint5 = vrep.simxGetObjectHandle(self.clientID, "yaw_joint4", vrep.simx_opmode_oneshot_wait)
        res, self.HandleJoint6 = vrep.simxGetObjectHandle(self.clientID, "yaw_joint5", vrep.simx_opmode_oneshot_wait)
        res, self.HandleJoint7 = vrep.simxGetObjectHandle(self.clientID, "yaw_joint6", vrep.simx_opmode_oneshot_wait)
        res, self.HandleJoint8 = vrep.simxGetObjectHandle(self.clientID, "yaw_joint7", vrep.simx_opmode_oneshot_wait)
        res, self.HandleHead = vrep.simxGetObjectHandle(self.clientID, "snake", vrep.simx_opmode_oneshot_wait)

        for training_time in range(training_times):
            self.current_train = training_time  # 当前训练次数
            for roll_out in range(roll_outs):
                self.current_roll_out = roll_out  # 当前轨迹数
                # 产生随机数，训练参数的变化量
                self.omega_delta1[roll_out] = numpy.random.normal(0.0, self.omega_gamma1, 1)
                self.omega_delta2[roll_out] = numpy.random.normal(0.0, self.omega_gamma2, 1)
                self.gamma_delta1[roll_out] = numpy.random.normal(0.0, self.gamma_gamma1, 1)
                self.gamma_delta2[roll_out] = numpy.random.normal(0.0, self.gamma_gamma2, 1)

                # 更新训练参数
                self.omega_roll1[roll_out] = self.omega01 + self.omega_delta1[roll_out]
                self.omega_roll2[roll_out] = self.omega02 + self.omega_delta2[roll_out]
                self.gamma_roll1[roll_out] = self.gamma01 + self.gamma_delta1[roll_out]
                self.gamma_roll2[roll_out] = self.gamma02 + self.gamma_delta2[roll_out]

                # 训练参数限幅
                if self.omega_roll1[roll_out] > 4 * pi:
                    self.omega_roll1[roll_out] = 4 * pi
                if self.omega_roll1[roll_out] < 0.5 * pi:
                    self.omega_roll1[roll_out] = 0.5 * pi

                if self.omega_roll2[roll_out] > 4 * pi:
                    self.omega_roll2[roll_out] = 4 * pi
                if self.omega_roll2[roll_out] < 0.5 * pi:
                    self.omega_roll2[roll_out] = 0.5 * pi

                if self.gamma_roll1[roll_out] > 0.12:
                    self.gamma_roll1[roll_out] = 0.12
                if self.gamma_roll1[roll_out] < -0.12:
                    self.gamma_roll1[roll_out] = -0.12
                if self.gamma_roll1[roll_out] > 0:
                    self.gamma_roll1[roll_out] = - self.gamma_roll1[roll_out]

                if self.gamma_roll2[roll_out] > 0.12:
                    self.gamma_roll2[roll_out] = 0.12
                if self.gamma_roll2[roll_out] < -0.12:
                    self.gamma_roll2[roll_out] = -0.12
                if self.gamma_roll2[roll_out] > 0:
                    self.gamma_roll2[roll_out] = - self.gamma_roll2[roll_out]

                # 记录每一条轨迹的随机参数
                self.omega_all1[self.current_train, self.current_roll_out] = self.omega_roll1[roll_out]
                self.omega_all2[self.current_train, self.current_roll_out] = self.omega_roll2[roll_out]
                self.gamma_all1[self.current_train, self.current_roll_out] = self.gamma_roll1[roll_out]
                self.gamma_all2[self.current_train, self.current_roll_out] = self.gamma_roll2[roll_out]

                print(self.omega_roll1[roll_out], self.omega_roll2[roll_out], self.gamma_roll1[roll_out],
                      self.gamma_roll2[roll_out])

                vrep.simxFinish(-1)  # 关闭所有可能的连接
                self.clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # 创建和vrep之间的通信线程
                vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_blocking)  # 开始仿真

                self.time_step()
                time.sleep(21)

            exponential_value_cost = numpy.zeros(roll_outs, dtype=numpy.float64)  # 代价指数
            probability_weighting = numpy.zeros(roll_outs, dtype=numpy.float64)  # 每条轨迹的概率
            for i in range(roll_outs):
                exponential_value_cost[i] = exp(
                    -self.constant1 * (self.cost[i] - self.cost.min()) / (self.cost.max() - self.cost.min()))
            for i in range(roll_outs):
                probability_weighting[i] = exponential_value_cost[i] / numpy.sum(exponential_value_cost)
            temp_delta_omega1 = 0
            temp_delta_omega2 = 0
            temp_delta_gamma1 = 0
            temp_delta_gamma2 = 0
            for i in range(roll_outs):
                temp_delta_omega1 = temp_delta_omega1 + probability_weighting[i] * self.omega_delta1[i]
                temp_delta_omega2 = temp_delta_omega2 + probability_weighting[i] * self.omega_delta2[i]
                temp_delta_gamma1 = temp_delta_gamma1 + probability_weighting[i] * self.gamma_delta1[i]
                temp_delta_gamma2 = temp_delta_gamma2 + probability_weighting[i] * self.gamma_delta2[i]

            # 更新参数
            self.omega01 = self.omega01 + temp_delta_omega1
            self.omega_train1[self.current_train] = self.omega01

            self.omega02 = self.omega02 + temp_delta_omega2
            self.omega_train2[self.current_train] = self.omega02

            self.gamma01 = self.gamma01 + temp_delta_gamma1
            self.gamma01 = -abs(self.gamma01)  # gamma1恒为负值
            self.gamma_train1[self.current_train] = self.gamma01

            self.gamma02 = self.gamma02 + temp_delta_gamma2
            self.gamma02 = -abs(self.gamma02)
            self.gamma_train2[self.current_train] = self.gamma02

            # 计算每次迭代后的损失函数
            for kk in range(5):
                self.PI2_counts = kk

                vrep.simxFinish(-1)  # 关闭所有可能的连接
                self.clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # 创建和vrep之间的通信线程
                vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_blocking)  # 开始仿真

                self.time_step_train()
                time.sleep(21)

            self.cost_train[self.current_train] = self.PI2_cost.min()


            if self.cost_train[self.current_train] >= 0.5:
                self.omega_gamma1 = 0.5
                self.omega_gamma2 = 0.5
                self.gamma_gamma1 = 0.02
                self.gamma_gamma2 = 0.02
            if self.cost_train[self.current_train] < 0.5 and self.cost_train[self.current_train] >= 0.1:
                self.omega_gamma2 = self.cost_train[self.current_train] * 0.5
                self.omega_gamma2 = self.cost_train[self.current_train] * 0.5
                self.gamma_gamma1 = self.cost_train[self.current_train] * 0.02
                self.gamma_gamma2 = self.cost_train[self.current_train] * 0.02
            if self.cost_train[self.current_train] < 0.1:
                self.omega_gamma1 = self.cost_train[self.current_train] * 0.5 / 2
                self.omega_gamma2 = self.cost_train[self.current_train] * 0.5 / 2
                self.gamma_gamma1 = self.cost_train[self.current_train] * 0.02 / 2
                self.gamma_gamma2 = self.cost_train[self.current_train] * 0.02 / 2

    def get_positon_from_vrep(self):  # 获取头部位置
        res, self.end_time_position = vrep.simxGetObjectPosition(self.clientID, self.HandleHead, -1,
                                                                 vrep.simx_opmode_oneshot_wait)  # 从vrep仿真环境下获取头部实时位置

    def satisfy_constraint(self):

        res, self.obstacle1_position = vrep.simxGetObjectPosition(self.clientID, self.HandleObstacle1, -1,
                                                                  vrep.simx_opmode_oneshot_wait)  # 从vrep仿真环境下获取头部实时位置
        res, self.obstacle2_position = vrep.simxGetObjectPosition(self.clientID, self.HandleObstacle2, -1,
                                                                  vrep.simx_opmode_oneshot_wait)  # 从vrep仿真环境下获取头部实时位置

        temp1 = abs(self.obstacle1_position[0] - self.init_obstacle1_position[0])
        temp2 = abs(self.obstacle1_position[1] - self.init_obstacle1_position[1])
        temp3 = abs(self.obstacle1_position[2] - self.init_obstacle1_position[2])
        temp4 = abs(self.obstacle2_position[0] - self.init_obstacle2_position[0])
        temp5 = abs(self.obstacle2_position[1] - self.init_obstacle2_position[1])
        temp6 = abs(self.obstacle2_position[2] - self.init_obstacle2_position[2])
        if temp1 > 0.005 or temp2 > 0.005 or temp3 > 0.005 or temp4 > 0.005 or temp5 > 0.005 or temp6 > 0.005:
            return False
        temp1 = self.motion_area_point1[0] - self.end_time_position[0]
        temp2 = self.motion_area_point1[1] - self.end_time_position[1]
        temp3 = self.end_time_position[0] - self.motion_area_point2[0]
        temp4 = self.end_time_position[1] - self.motion_area_point2[1]
        if temp1 > 0 or temp2 > 0 or temp3 > 0 or temp4 > 0:
            return False

        return True

    def time_step_train(self):
        if self.current_time >= self.T:  # 到达终止时刻
            self.get_positon_from_vrep()  # 获取终止时刻头部位置
            self.PI2_cost[self.PI2_counts] = sqrt(pow(self.goal[0] - self.end_time_position[0], 2) + pow(self.goal[1] - self.end_time_position[1],2))  # 计算代价

            # 越界或者碰到障碍物
            flag_temp = self.satisfy_constraint()
            if flag_temp == False:
                self.PI2_cost[self.PI2_counts] = self.max_cost

            print(self.PI2_cost[self.PI2_counts])

            ##在vrep里让机器人立马回到初始位姿
            vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)
            vrep.simxFinish(self.clientID)
            self.current_time = 0
            return

        # 计算每个时间步关节角度
        for i in range(N):
            if self.current_time < self.T1:
                self.joint_angle[N - 1 - i] = -self.A1 * sin(
                    self.omega_train1[self.current_train] * self.current_time + (i - 1) * self.beta1) + \
                                              self.gamma_train1[self.current_train]
            if self.current_time >= self.T1:
                self.joint_angle[N - 1 - i] = -self.A2 * sin(
                    self.omega_train2[self.current_train] * self.current_time + (i - 1) * self.beta2) + \
                                              self.gamma_train2[self.current_train]
            # 关节角度限幅
            if self.joint_angle[i] > 60 / 180 * pi:
                self.joint_angle[i] = 60 / 180 * pi
            if self.joint_angle[i] < -60 / 180 * pi:
                self.joint_angle[i] = -60 / 180 * pi

        vrep.simxSetJointTargetPosition(self.clientID, self.HandleJoint1, self.joint_angle[0],
                                        vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(self.clientID, self.HandleJoint2, self.joint_angle[1],
                                        vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(self.clientID, self.HandleJoint3, self.joint_angle[2],
                                        vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(self.clientID, self.HandleJoint4, self.joint_angle[3],
                                        vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(self.clientID, self.HandleJoint5, self.joint_angle[4],
                                        vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(self.clientID, self.HandleJoint6, self.joint_angle[5],
                                        vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(self.clientID, self.HandleJoint7, self.joint_angle[6],
                                        vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(self.clientID, self.HandleJoint8, self.joint_angle[7],
                                        vrep.simx_opmode_oneshot)

        self.current_time += self.dt
        timer = threading.Timer(self.dt, self.time_step_train)
        timer.start()

    def time_step(self):
        if self.current_time >= self.T:  # 到达终止时刻
            self.get_positon_from_vrep()  # 获取终止时刻头部位置
            self.cost[self.current_roll_out] = sqrt(pow(self.goal[0] - self.end_time_position[0], 2) + pow(self.goal[1] - self.end_time_position[1],2))  # 计算代价

            # 越界或者碰到障碍物
            flag_temp = self.satisfy_constraint()
            if flag_temp == False:
                self.cost[self.current_roll_out] = self.max_cost

            print(self.cost[self.current_roll_out])

            ##在vrep里让机器人立马回到初始位姿
            vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)
            vrep.simxFinish(self.clientID)
            self.current_time = 0
            return

        # 计算每个时间步关节角度
        for i in range(N):
            if self.current_time < self.T1:
                self.joint_angle[N - 1 - i] = -self.A1 * sin(
                    self.omega_roll1[self.current_roll_out] * self.current_time + (i - 1) * self.beta1) + \
                                              self.gamma_roll1[self.current_roll_out]
            if self.current_time >= self.T1:
                self.joint_angle[N - 1 - i] = -self.A2 * sin(
                    self.omega_roll2[self.current_roll_out] * self.current_time + (i - 1) * self.beta2) + \
                                              self.gamma_roll2[self.current_roll_out]
            # 关节角度限幅
            if self.joint_angle[i] > 60 / 180 * pi:
                self.joint_angle[i] = 60 / 180 * pi
            if self.joint_angle[i] < -60 / 180 * pi:
                self.joint_angle[i] = -60 / 180 * pi

        vrep.simxSetJointTargetPosition(self.clientID, self.HandleJoint1, self.joint_angle[0],
                                        vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(self.clientID, self.HandleJoint2, self.joint_angle[1],
                                        vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(self.clientID, self.HandleJoint3, self.joint_angle[2],
                                        vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(self.clientID, self.HandleJoint4, self.joint_angle[3],
                                        vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(self.clientID, self.HandleJoint5, self.joint_angle[4],
                                        vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(self.clientID, self.HandleJoint6, self.joint_angle[5],
                                        vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(self.clientID, self.HandleJoint7, self.joint_angle[6],
                                        vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(self.clientID, self.HandleJoint8, self.joint_angle[7],
                                        vrep.simx_opmode_oneshot)

        self.current_time += self.dt
        timer = threading.Timer(self.dt, self.time_step)
        timer.start()


if __name__ == '__main__':
    mPI2 = PI2()
    mPI2.sim_start()
    # 保存数据
    numpy.save('omega_all1.npy', mPI2.omega_all1)  # n_trains * n_rolls
    numpy.save('gamma_all1.npy', mPI2.gamma_all1)  # n_trains * n_rolls
    numpy.save('omega_all2.npy', mPI2.omega_all2)  # n_trains * n_rolls
    numpy.save('gamma_all2.npy', mPI2.gamma_all2)  # n_trains * n_rolls

    numpy.save('omega_train1.npy', mPI2.omega_train1)  # n_trains
    numpy.save('gamma_train1.npy', mPI2.gamma_train1)  # n_trains
    numpy.save('omega_train2.npy', mPI2.omega_train2)  # n_trains
    numpy.save('gamma_train2.npy', mPI2.gamma_train2)  # n_trains

    numpy.save('cost_all.npy', mPI2.cost_all)  # n_trains * n_rolls
    numpy.save('cost_train.npy', mPI2.cost_train)  # n_trains

