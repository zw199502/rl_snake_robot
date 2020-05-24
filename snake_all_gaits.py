#!/usr/bin/env python
# -*- coding: utf-8 -*-

###MX106T,Min Angle: 90/1024, 270/3072




import threading

import rospy
import numpy as np
import os

import time
from set_initial_state import reset_state
from snake_pipeline import pipeline
from std_msgs.msg import Float64

import sys
import tty
import termios

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)


def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

N = 16 # the number of actuator
rospy.init_node('DDPG', anonymous=True)
PL = pipeline()
RS = reset_state()

class MX106T_control:
    def __init__(self):

        self.control_angle = np.zeros((N, 1), dtype=np.float)  # control angles; initial: 0 degree;
      
        # whether actuators are enabled, 0: disable   1: enable
        self.actuator_enable = 0
        # start or pause motion, 0: pause     1: start
        self.motion_start = 0
        # control mode
        # 0: do nothing
        # 1: sidewinding
        # 2: rolling
        # 3: climbing pipeline
        # 4: serpentining
        # 5: travelling waving
        self.control_mode = 0
        
        # parameters of snake curve
        # sidewinding
        self.sidewinding_A_yaw = np.pi / 8
        self.sidewinding_A_pitch = np.pi / 10
        self.sidewinding_omega_temporal = np.pi / 2
        self.sidewinding_omega_spatial = np.pi / 2
        self.sidewinding_delta = np.pi / 12
        self.sidewinding_phi = 0
        self.sidewinding_turning_factor = 0.0
        self.sidewinding_motion_direction = 0 # 0: left    1: right
        self.sidewinding_speed = 1.0

        # rolling
        self.rolling_A_yaw = np.pi / 12
        self.rolling_A_pitch = np.pi / 12
        self.rolling_omega_temporal = 0
        self.rolling_omega_temporal = np.pi / 2
        self.rolling_omega_spatial = np.pi / 10
        self.rolling_phi = 0
        self.rolling_motion_direction = 0 # 0: left    1: right
        self.rolling_speed = 1.0

        # climbing pipeline
        self.climbing_A_yaw = 2.15 * np.pi / 5
        self.climbing_A_pitch = 2.15 * np.pi / 5
        self.climbing_omega_temporal = np.pi / 6
        self.climbing_omega_spatial = np.pi / 15
        self.climbing_phi = 0
        self.climbing_motion_direction = 0 # 0: up    1: down
        self.climbing_speed = 1.0

        # serpentining
        self.serpentining_A_yaw = np.pi / 6
        self.serpentining_omega_temporal = np.pi / 2
        self.serpentining_omega_spatial = np.pi / 2
        self.serpentining_phi = 0
        self.serpentining_turning_factor = 0.0
        self.serpentining_motion_direction = 0 # 0: left    1: right
        self.serpentining_speed = 1.0

        # travelling
        self.travelling_A_pitch = np.pi / 8
        self.travelling_omega_temporal = np.pi / 2
        self.travelling_omega_spatial = np.pi / 2
        self.travelling_motion_direction = 0 # 0: forward    1: backward
        self.travelling_phi = 0
        self.travelling_speed = 1.0

        ##locomotion
        self.dt = 0.02   # control period/s
        self.sidewinding_current_time = 0   # current time/s 
        self.rolling_current_time = 0   # current time/s
        self.climbing_current_time = 0   # current time/s
        self.serpentining_current_time = 0   # current time/s
        self.travelling_current_time = 0   # current time/s
        self.control_gui()

    def control_gui(self):
        print('')
        print("Please input command: ")
        print('')
        print("q: enable actuator")
        print("w: start")
        print("e: pause")
        print("r: reset")
        print("t: disable actuator")
        print("y: terminate process")
        # motion modes
        print('')
        print("a: sidewinding")
        print("s: rolling")
        print("d: climbing pipeline")
        print("f: serpentining")
        print("g: travelling waving")
        # speed control
        print('')
        print("8: speed up")
        print("2: speed down")
        # turning control
        # sidewinding, rolling and serpentining
        print('')
        print("7: turn left")
        print("9: turn right")
        # direction control
        # climbing pipeline: up and down
        # travelling waving: forward and backward
        # sidewinding, serpentining, and rolling: left and right
        print('')
        print("4: up/forward/left")
        print("6: down/backward/right")
        # get position and load
        print('')
        print("p: get position and load")
        # twining form
        print('')
        print("l: rough twining")
        print("m: tight twining")
        # manual control for joint 0,1,3
        print('')
        print("z: joint0 left")
        print("x: joint0 right")
        print("c: joint1 up")
        print("v: joint1 down")
        print("b: joint3 up")
        print("n: joint3 down")
        # twining adjustment
        print('')
        print("j: twining loosen")
        print("k: twining tighten")


    def reset_MX106T(self):  # reset MX106T as initial state
        self.sidewinding_current_time = 0
        self.rolling_current_time = 0
        self.climbing_current_time = 0
        self.serpentiningcurrent_time = 0
        self.travelling_current_time = 0

        self.sidewinding_motion_direction = 0 
        self.sidewinding_speed = 1.0

        self.rolling_motion_direction = 0 
        self.rolling_speed = 1.0

        self.climbing_motion_direction = 0 
        self.climbing_speed = 1.0

        self.serpentining_motion_direction = 0 
        self.serpentining_speed = 1.0

        self.travelling_motion_direction = 0
        self.travelling_speed = 1.0

        for i in range(N):
            self.control_angle[i] = 0  # set joint angle as zero
        #self.send_control_packet()
        s = RS.set_initial_state()


    def send_control_packet(self):
        PL.step(self.control_angle)

    def speed_up(self):   # speed up
        if self.control_mode == 1:
            self.sidewinding_speed = self.sidewinding_speed + 0.1
            if self.sidewinding_speed > 10:
                self.sidewinding_speed = 10
            self.set_omega()
            print("current speed: %s" %(self.sidewinding_speed))
        
        if self.control_mode == 2:
            self.rolling_speed = self.rolling_speed + 0.1
            if self.rolling_speed > 10:
                self.rolling_speed = 10
            self.set_omega()
            print("current speed: %s" %(self.rolling_speed))

        if self.control_mode == 3:
            self.climbing_speed = self.climbing_speed + 0.1
            if self.climbing_speed > 10:
                self.climbing_speed = 10
            self.set_omega()
            print("current speed: %s" %(self.climbing_speed))

        if self.control_mode == 4:
            self.serpentining_speed = self.serpentining_speed + 0.1
            if self.serpentining_speed > 10:
                self.serpentining_speed = 10
            self.set_omega()
            print("current speed: %s" %(self.serpentining_speed))

        if self.control_mode == 5:
            self.travelling_speed = self.travelling_speed + 0.1
            if self.travelling_speed > 10:
                self.travelling_speed = 10
            self.set_omega()
            print("current speed: %s" %(self.travelling_speed))

    def speed_down(self):   # speed down
        if self.control_mode == 1:
            self.sidewinding_speed = self.sidewinding_speed - 0.1
            if self.sidewinding_speed < 0.1:
                self.sidewinding_speed = 0.1
            self.set_omega()
            print("current speed: %s" %(self.sidewinding_speed))
        
        if self.control_mode == 2:
            self.rolling_speed = self.rolling_speed - 0.1
            if self.rolling_speed < 0.1:
                self.rolling_speed = 0.1
            self.set_omega()
            print("current speed: %s" %(self.rolling_speed))

        if self.control_mode == 3:
            self.climbing_speed = self.climbing_speed - 0.1
            if self.climbing_speed < 0.1:
                self.climbing_speed = 0.1
            self.set_omega()
            print("current speed: %s" %(self.climbing_speed))

        if self.control_mode == 4:
            self.serpentining_speed = self.serpentining_speed - 0.1
            if self.serpentining_speed < 0.1:
                self.serpentining_speed = 0.1
            self.set_omega()
            print("current speed: %s" %(self.serpentining_speed))

        if self.control_mode == 5:
            self.travelling_speed = self.travelling_speed - 0.1
            if self.travelling_speed < 0.1:
                self.travelling_speed = 0.1
            self.set_omega()
            print("current speed: %s" %(self.travelling_speed))

    def factor_increase(self):
        if self.control_mode == 1: # sidewinding
            
            self.sidewinding_turning_factor = self.sidewinding_turning_factor + 0.05
            if self.sidewinding_turning_factor > 0.3:
                self.sidewinding_turning_factor = 0.3
            print("current turning factor of sidewinding: %s" %(self.sidewinding_turning_factor))

        if self.control_mode == 4: # serpentining
            
            self.serpentining_turning_factor = self.serpentining_turning_factor + 0.05
            if self.serpentining_turning_factor > 0.3:
                self.serpentining_turning_factor = 0.3
            print("current turning factor of serpentining: %s" %(self.serpentining_turning_factor))
            
    def factor_decrease(self):
        if self.control_mode == 1: # sidewinding
            
            self.sidewinding_turning_factor = self.sidewinding_turning_factor - 0.05
            if self.sidewinding_turning_factor < -0.3:
                self.sidewinding_turning_factor = -0.3
            print("current turning factor of sidewinding: %s" %(self.sidewinding_turning_factor))

        if self.control_mode == 4: # serpentining
            
            self.serpentining_turning_factor = self.serpentining_turning_factor - 0.05
            if self.serpentining_turning_factor < -0.3:
                self.serpentining_turning_factor = -0.3
            print("current turning factor of serpentining: %s" %(self.serpentining_turning_factor))

    def left_forward_up(self):
        if self.control_mode == 1:
            if self.sidewinding_motion_direction == 1:
                self.sidewinding_motion_direction = 0 
                self.sidewinding_phi = self.sidewinding_omega_temporal * self.sidewinding_current_time + self.sidewinding_phi
                self.sidewinding_current_time = 0
                self.set_omega()
                print('serpentining left')
        if self.control_mode == 2:
            if self.rolling_motion_direction == 1:
                self.rolling_motion_direction = 0
                self.rolling_phi = self.rolling_omega_temporal * self.rolling_current_time + self.rolling_phi
                self.rolling_current_time = 0
                self.set_omega()
                print('rolling left')
        if self.control_mode == 3:
            if self.climbing_motion_direction == 1:
                self.climbing_motion_direction = 0
                self.climbing_phi = self.climbing_omega_temporal * self.climbing_current_time + self.climbing_phi
                self.climbing_current_time = 0
                self.set_omega()
                print('climbing up')
        if self.control_mode == 4:
            if self.serpentining_motion_direction == 1:
                self.serpentining_motion_direction = 0
                self.serpentining_phi = self.serpentining_omega_temporal * self.serpentining_current_time + self.serpentining_phi
                self.serpentining_current_time = 0
                self.set_omega()
                print('serpentining left')
        if self.control_mode == 5:
            if self.travelling_motion_direction == 1:
                self.travelling_motion_direction = 0
                self.travelling_phi = self.travelling_omega_temporal * self.travelling_current_time + self.travelling_phi
                self.travelling_current_time = 0
                self.set_omega()
                print('travelling forward')

    def right_backward_down(self):
        if self.control_mode == 1:
            if self.sidewinding_motion_direction == 0:
                self.sidewinding_motion_direction = 1
                self.sidewinding_phi = self.sidewinding_omega_temporal * self.sidewinding_current_time + self.sidewinding_phi
                self.sidewinding_current_time = 0
                self.set_omega()
                print('serpentining right')
        if self.control_mode == 2:
            if self.rolling_motion_direction == 0:
                self.rolling_motion_direction = 1
                self.rolling_phi = self.rolling_omega_temporal * self.rolling_current_time + self.rolling_phi
                self.rolling_current_time = 0
                self.set_omega()
                print('rolling right')
        if self.control_mode == 3:
            if self.climbing_motion_direction == 0:
                self.climbing_motion_direction = 1
                self.climbing_phi = self.climbing_omega_temporal * self.climbing_current_time + self.climbing_phi
                self.climbing_current_time = 0
                self.set_omega()
                print('climbing down')
        if self.control_mode == 4:
            if self.serpentining_motion_direction == 0:
                self.serpentining_motion_direction = 1
                self.serpentining_phi = self.serpentining_omega_temporal * self.serpentining_current_time + self.serpentining_phi
                self.serpentining_current_time = 0
                self.set_omega()
                print('serpentining right')
        if self.control_mode == 5:
            if self.travelling_motion_direction == 0:
                self.travelling_motion_direction = 1
                self.travelling_phi = self.travelling_omega_temporal * self.travelling_current_time + self.travelling_phi
                self.travelling_current_time = 0
                self.set_omega()
                print('travelling backward')

    def set_omega(self):
        if self.control_mode == 1:
            if self.sidewinding_motion_direction == 0:
                self.sidewinding_omega_temporal = self.sidewinding_speed * np.pi / 2
            if self.sidewinding_motion_direction == 1:
                self.sidewinding_omega_temporal = -self.sidewinding_speed * np.pi / 2

            
        if self.control_mode == 2:
            if self.rolling_motion_direction == 0:
                self.rolling_omega_temporal = self.rolling_speed * np.pi / 2
            if self.rolling_motion_direction == 1:
                self.rolling_omega_temporal = -self.rolling_speed * np.pi / 2
                
        if self.control_mode == 3:
            if self.climbing_motion_direction == 0:
                self.climbing_omega_temporal = self.climbing_speed * np.pi / 6
            if self.climbing_motion_direction == 1:
                self.climbing_omega_temporal = -self.climbing_speed * np.pi / 6
            
            
        if self.control_mode == 4:
            if self.serpentining_motion_direction == 0:
                self.serpentining_omega_temporal = self.serpentining_speed * np.pi / 2
            if self.serpentining_motion_direction == 1:
                self.serpentining_omega_temporal = -self.serpentining_speed * np.pi / 2
            
        if self.control_mode == 5:
            if self.travelling_motion_direction == 0:
                self.travelling_omega_temporal = self.travelling_speed * np.pi / 2
            if self.travelling_motion_direction == 1:
                self.travelling_omega_temporal = -self.travelling_speed * np.pi / 2
            

    def snake_curve(self):  # different snake curves
        if self.motion_start == 0:
            return

        if self.control_mode == 1: # sidewinding
            for i in range(8):
                angle_rad = self.sidewinding_A_yaw * np.sin(self.sidewinding_omega_temporal * self.sidewinding_current_time \
                + self.sidewinding_omega_spatial * i + self.sidewinding_delta + self.sidewinding_phi) + self.sidewinding_turning_factor
                self.control_angle[2 * i] = angle_rad
                angle_rad = self.sidewinding_A_pitch * np.sin(self.sidewinding_omega_temporal * self.sidewinding_current_time \
                + self.sidewinding_omega_spatial * i + self.sidewinding_phi)
                self.control_angle[2 * i + 1] = angle_rad
            
            self.sidewinding_current_time = self.sidewinding_current_time + self.dt

        if self.control_mode == 2: # rolling
            for i in range(8):
                angle_rad = self.rolling_A_yaw * np.sin(self.rolling_omega_temporal * self.rolling_current_time + self.rolling_phi)
                self.control_angle[2 * i] = angle_rad
                angle_rad = self.rolling_A_pitch * np.cos(self.rolling_omega_temporal * self.rolling_current_time + self.rolling_phi)
                self.control_angle[2 * i + 1] = angle_rad
            
            self.rolling_current_time = self.rolling_current_time + self.dt

        if self.control_mode == 3: # climbing
            for i in range(8):
                angle_rad = self.climbing_A_yaw * np.sin(self.climbing_omega_temporal * self.climbing_current_time \
                + self.climbing_omega_spatial * i + self.climbing_phi)
                self.control_angle[2 * i] = angle_rad
                angle_rad = self.climbing_A_pitch * np.cos(self.climbing_omega_temporal * self.climbing_current_time \
                + self.climbing_omega_spatial * i + self.climbing_phi)
                self.control_angle[2 * i + 1] = angle_rad
            
            self.climbing_current_time = self.climbing_current_time + self.dt
            self.read_position()
            current_angle = self.position_read * 360 / 4096 - 180 * np.ones((N, 1), dtype=np.float)
            error = current_angle - self.control_angle  #unit: degree
            min_error = np.min(error)
            max_error = np.max(error)
            time.sleep(self.dt)
            while (max_error > 5 or min_error < -5):
                self.send_control_packet()
                self.read_position()
                current_angle = self.position_read * 360 / 4096 - 180 * np.ones((N, 1), dtype=np.float)
                error = current_angle - self.control_angle  #unit: degree
                min_error = np.min(error)
                max_error = np.max(error)
                time.sleep(self.dt)

        if self.control_mode == 4: # serpentining
            for i in range(8):
                angle_rad = self.serpentining_A_yaw * np.sin(self.serpentining_omega_temporal * self.serpentining_current_time \
                + self.serpentining_omega_spatial * i + self.serpentining_phi) + self.serpentining_turning_factor
                self.control_angle[2 * i] = angle_rad 
                self.control_angle[2 * i + 1] = 0.0
            
            self.serpentining_current_time = self.serpentining_current_time + self.dt

        if self.control_mode == 5: # travelling
            for i in range(7):
                angle_rad = self.travelling_A_pitch * np.sin(self.travelling_omega_temporal * self.travelling_current_time \
                + self.travelling_omega_spatial * i + self.travelling_phi)
                self.control_angle[2 * i + 1] = angle_rad 
            
            self.travelling_current_time = self.travelling_current_time + self.dt

        self.send_control_packet()
        motion_timer = threading.Timer(self.dt, self.snake_curve)
        motion_timer.start()

    def pipeline_initial(self):  # press 'm', search initial climbing_phi
        
        for i in range(8):
            angle_rad = 2.13 * np.pi / 5 * np.sin(self.climbing_omega_spatial * i + self.climbing_phi)
            self.control_angle[2 * i] = angle_rad * 180.0 / np.pi
            angle_rad = 2.13 * np.pi / 5 * np.cos(self.climbing_omega_spatial * i + self.climbing_phi)
            self.control_angle[2 * i + 1] = angle_rad * 180.0 / np.pi
        self.send_control_packet()
        time.sleep(0.5)

    def pipeline_form(self):  # press 'l', search initial climbing_phi
        time.sleep(5)
        A_temp = 0
        while A_temp < (self.climbing_A_yaw - 0.15):
            A_temp = A_temp + 0.05
            for i in range(8):
                angle_rad = A_temp * np.sin(self.climbing_omega_spatial * i + self.climbing_phi)
                self.control_angle[2 * i] = angle_rad * 180.0 / np.pi
                angle_rad = A_temp * np.cos(self.climbing_omega_spatial * i + self.climbing_phi)
                self.control_angle[2 * i + 1] = angle_rad * 180.0 / np.pi
            self.send_control_packet()
            time.sleep(0.5)

    def joint0_left(self):
        self.control_angle[0] = self.control_angle[0] + 1
        self.send_control_packet()
        print("joint0: %d" %(self.control_angle[0]))

    def joint0_right(self):
        self.control_angle[0] = self.control_angle[0] - 1
        self.send_control_packet()
        print("joint0: %d" %(self.control_angle[0]))

    def joint1_up(self):
        self.control_angle[1] = self.control_angle[1] - 1
        self.send_control_packet()
        print("joint1: %d" %(self.control_angle[1]))

    def joint1_down(self):
        self.control_angle[1] = self.control_angle[1] + 1
        self.send_control_packet()
        print("joint1: %d" %(self.control_angle[1]))

    def joint3_up(self):
        self.control_angle[3] = self.control_angle[3] - 1
        self.send_control_packet()
        print("joint3: %d" %(self.control_angle[3]))

    def joint3_down(self):
        self.control_angle[3] = self.control_angle[3] + 1
        self.send_control_packet()
        print("joint3: %d" %(self.control_angle[3]))

    def twining_loosen(self):
        self.climbing_A_pitch = self.climbing_A_pitch + 0.01
        self.climbing_A_yaw = self.climbing_A_yaw + 0.01
        print("climbing_A_yaw: %s" %(self.climbing_A_yaw))
        print("climbing_A_pitch: %s" %(self.climbing_A_pitch))

    def twining_tighten(self):
        self.climbing_A_pitch = self.climbing_A_pitch - 0.01
        self.climbing_A_yaw = self.climbing_A_yaw - 0.01
        print("climbing_A_yaw: %s" %(self.climbing_A_yaw))
        print("climbing_A_pitch: %s" %(self.climbing_A_pitch))


if __name__ == '__main__':
    snake = MX106T_control()
    ch = ''
    while True:
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        if ch == 'q':
            if snake.actuator_enable == 0:
                snake.actuator_enable = 1
                
                print('Enable actuator')
        if ch == 'w':
            if snake.actuator_enable == 1:
                if snake.motion_start == 0:
                    snake.motion_start = 1
                    snake.snake_curve()
                    print('Start motion')
        if ch == 'e':
            if snake.actuator_enable == 1:
                if snake.motion_start == 1:
                    snake.motion_start = 0
                    print('Pause motion')
        if ch == 'r':  #reset to initial state
            if snake.actuator_enable == 1 and snake.motion_start == 0:
                snake.reset_MX106T()  # reset
                print('Reset')
        if ch == 't':
            if snake.actuator_enable == 1:
                snake.actuator_enable = 0
                
                print('Disable actuator')
        if ch == 'y':
            if snake.actuator_enable == 0:
                print('Terminate process')
                sys.exit()
        
        if ch == 'a':
            if snake.actuator_enable == 1 and snake.motion_start == 0:
                snake.control_mode = 1
                print('sidewinding')
        if ch == 's':
            if snake.actuator_enable == 1 and snake.motion_start == 0:
                snake.control_mode = 2
                print('rolling')
        if ch == 'd':
            if snake.actuator_enable == 1 and snake.motion_start == 0:
                snake.control_mode = 3
                print('climbing pipeline')
        if ch == 'f':
            if snake.actuator_enable == 1 and snake.motion_start == 0:
                snake.control_mode = 4
                print('serpentining')
        if ch == 'g':
            if snake.actuator_enable == 1 and snake.motion_start == 0:
                snake.control_mode = 5
                print('travelling waving')
            

       
        if ch == '8':
            if snake.actuator_enable == 1:
                snake.speed_up()
                print('Speed up')
        if ch == '2':
            if snake.actuator_enable == 1:
                snake.speed_down()
                print('Speed down')

        if ch == '9':
            if snake.actuator_enable == 1:
                snake.factor_increase()
                print('Increase turning factor')
        if ch == '7':
            if snake.actuator_enable == 1:
                snake.factor_decrease()
                print('Decrease turning factor')

        if ch == '4':
            if snake.actuator_enable == 1:
                snake.left_forward_up()
                print('Left/Forward/Up')
        if ch == '6':
            if snake.actuator_enable == 1:
                snake.right_backward_down()
                print('Right/Backward/Down')

        if ch == 'p':
            if snake.actuator_enable == 1:
                snake.get_position_load()
                
                print('angle/rad:')
                print(snake.current_angle_read)
                print('load:')
                print(snake.current_load_read)

        if ch == 'l':
            if snake.actuator_enable == 1:
                snake.pipeline_form()
                print('maintain climing form')

        if ch == 'm':
            if snake.actuator_enable == 1:
                snake.pipeline_initial()
                print('initial climing form')

        if ch == 'z':
            if snake.actuator_enable == 1:
                snake.joint0_left()

        if ch == 'x':
            if snake.actuator_enable == 1:
                snake.joint0_right()

        if ch == 'c':
            if snake.actuator_enable == 1:
                snake.joint1_up()

        if ch == 'v':
            if snake.actuator_enable == 1:
                snake.joint1_down()

        if ch == 'b':
            if snake.actuator_enable == 1:
                snake.joint3_up()

        if ch == 'n':
            if snake.actuator_enable == 1:
                snake.joint3_down()

        if ch == 'j':
            if snake.actuator_enable == 1:
                snake.twining_loosen()

        if ch == 'k':
            if snake.actuator_enable == 1:
                snake.twining_tighten()

        ch = ''



