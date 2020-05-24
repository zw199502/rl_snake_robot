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
import threading

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)

np.random.seed(1)

rospy.init_node('DDPG', anonymous=True)

N = 16

class Climb:
    def __init__(self):
        self.PL = pipeline()
        self.RS = reset_state()

        self.control_angle = np.zeros((N, 1), dtype=np.float)  # joint angle, unit: rad, range: -pi/2 to pi/2
        self.read_angle = np.zeros((N, 1), dtype=np.float)  # angle reading from Gazebo
        self.read_effort = np.zeros((N, 1), dtype=np.float) # joint effort


        # whether actuators are enabled, 0: disable   1: enable
        self.actuator_enable = 0
        # start or pause motion, 0: pause     1: start
        self.motion_start = 0
        # control mode

        # climbing pipeline
        self.climbing_A_yaw = 2.10 * np.pi / 5
        self.climbing_A_pitch = 2.10 * np.pi / 5
        self.climbing_omega_temporal = np.pi / 20
        self.climbing_omega_spatial = np.pi / 15
        self.climbing_phi = 0.0
        self.climbing_motion_direction = 0 # 0: up    1: down
        self.climbing_speed = 1.0

        ##locomotion
        self.dt_climbing = 0.05 # climbing control period

        self.period = 0.1
    
        self.climbing_current_time = 0   # current time/s
        
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
        
        # speed control
        print('')
        print("8: speed up")
        print("2: speed down")
        
        # direction control
        print('')
        print("4: down")
        print("6: up")
        # get position and load
        print('')
        print("p: get position and load")
        # twining form
        print('')
        print("l: rough twining")
        print("m: tight twining")
       
        # twining adjustment
        print('')
        print("j: twining loosen")
        print("k: twining tighten")

    def send_control_packet(self):
        self.PL.step(self.control_angle, self.period)

    def get_position(self):
        self.read_angle = self.PL.read_joint_angle

    def get_effort(self):
        self.read_effort = self.PL.read_joint_effort

    def get_position_effort(self):  # press 'p', get position of each joint
        self.get_position()
        self.read_effort()

    def speed_up(self):   # speed up
       
        
        self.climbing_speed = self.climbing_speed + 0.1
        if self.climbing_speed > 10:
            self.climbing_speed = 10
        self.set_omega()
        print("current speed: %s" %(self.climbing_speed))

    def speed_down(self):   # speed down

        
        self.climbing_speed = self.climbing_speed - 0.1
        if self.climbing_speed < 0.1:
            self.climbing_speed = 0.1
        self.set_omega()
        print("current speed: %s" %(self.climbing_speed))

    def up(self):
        
        if self.climbing_motion_direction == 1:
            self.climbing_motion_direction = 0
            self.climbing_phi = self.climbing_omega_temporal * self.climbing_current_time + self.climbing_phi
            self.climbing_current_time = 0
            self.set_omega()
            print('climbing up')

    def down(self):
       
        
        if self.climbing_motion_direction == 0:
            self.climbing_motion_direction = 1
            self.climbing_phi = self.climbing_omega_temporal * self.climbing_current_time + self.climbing_phi
            self.climbing_current_time = 0
            self.set_omega()
            print('climbing down')
      

    def set_omega(self):
                
        
        if self.climbing_motion_direction == 0:
            self.climbing_omega_temporal = self.climbing_speed * np.pi / 20
        if self.climbing_motion_direction == 1:
            self.climbing_omega_temporal = -self.climbing_speed * np.pi / 20

    def climbing_curve(self): # climbing mode
        if self.motion_start == 0:
            return
        
        phi0 = self.climbing_omega_temporal * self.climbing_current_time
        phi0_mod = phi0 - (int(phi0 / 1.55)) * 1.55
        A_yaw0 = 2.11 * np.pi / 5
        A_pitch0 = 2.11 * np.pi / 5

        for i in range(8):
            
            # print(phi0_mod)
            # if phi0_mod > 1.2:

            #     A_yaw0 = 2.20 * np.pi / 5
            #     A_pitch0 = 2.20 * np.pi / 5

            # if phi0_mod > 1.0 and phi0_mod <= 1.2:
    
            #     A_yaw0 = 2.18 * np.pi / 5
            #     A_pitch0 = 2.18 * np.pi / 5

            # if phi0_mod > 0.8 and phi0_mod <= 1.0:
        
            #     A_yaw0 = 2.16 * np.pi / 5
            #     A_pitch0 = 2.16 * np.pi / 5

            # if phi0_mod > 0.5 and phi0_mod <= 0.08:
            
            #     A_yaw0 = 2.14 * np.pi / 5
            #     A_pitch0 = 2.14 * np.pi / 5

            # if phi0_mod > 0.2 and phi0_mod <= 0.5:
                
            #     A_yaw0 = 2.12 * np.pi / 5
            #     A_pitch0 = 2.12 * np.pi / 5

            # if phi0_mod >= 0.0 and phi0_mod <= 0.2:
            
            #     A_yaw0 = 2.11 * np.pi / 5
            #     A_pitch0 = 2.11 * np.pi / 5

            angle_rad = A_yaw0 * np.sin(phi0 + self.climbing_omega_spatial * i + self.climbing_phi)
            self.control_angle[2 * i] = angle_rad
                
            angle_rad = A_pitch0 * np.cos(phi0 + self.climbing_omega_spatial * i + self.climbing_phi)
            self.control_angle[2 * i + 1] = angle_rad
            
        self.climbing_current_time = self.climbing_current_time + self.dt_climbing

        self.send_control_packet()
        motion_climbing_timer = threading.Timer(self.dt_climbing, self.climbing_curve)
        motion_climbing_timer.start()

    def pipeline_initial(self):  # press 'm', search initial climbing_phi
        
        for i in range(8):
            angle_rad = 2.10 * np.pi / 5 * np.sin(self.climbing_omega_spatial * i + self.climbing_phi)
            self.control_angle[2 * i] = angle_rad
            angle_rad = 2.10 * np.pi / 5 * np.cos(self.climbing_omega_spatial * i + self.climbing_phi)
            self.control_angle[2 * i + 1] = angle_rad
        self.send_control_packet()
       
    def pipeline_form(self):  # press 'l', search initial climbing_phi
        time.sleep(0.5)
        A_temp = 0
        i = 0
        while A_temp < (self.climbing_A_yaw - 0.11):
            A_temp = A_temp + 0.02
            
            angle_rad = A_temp * np.sin(self.climbing_omega_spatial * i + self.climbing_phi)
            self.control_angle[2 * i] = angle_rad 
            angle_rad = A_temp * np.cos(self.climbing_omega_spatial * i + self.climbing_phi)
            self.control_angle[2 * i + 1] = angle_rad

            angle_rad = A_temp * np.sin(self.climbing_omega_spatial * (7 - i) + self.climbing_phi)
            self.control_angle[2 * (7 - i)] = angle_rad 
            angle_rad = A_temp * np.cos(self.climbing_omega_spatial * (7 - i) + self.climbing_phi)
            self.control_angle[2 * (7 - i) + 1] = angle_rad

            self.send_control_packet()

        A_temp = 0
        i = 1
        while A_temp < (self.climbing_A_yaw - 0.11):
            A_temp = A_temp + 0.02
            
            angle_rad = A_temp * np.sin(self.climbing_omega_spatial * i + self.climbing_phi)
            self.control_angle[2 * i] = angle_rad 
            angle_rad = A_temp * np.cos(self.climbing_omega_spatial * i + self.climbing_phi)
            self.control_angle[2 * i + 1] = angle_rad

            angle_rad = A_temp * np.sin(self.climbing_omega_spatial * (7 - i) + self.climbing_phi)
            self.control_angle[2 * (7 - i)] = angle_rad 
            angle_rad = A_temp * np.cos(self.climbing_omega_spatial * (7 - i) + self.climbing_phi)
            self.control_angle[2 * (7 - i) + 1] = angle_rad

            self.send_control_packet()

        A_temp = 0
        i = 2
        while A_temp < (self.climbing_A_yaw - 0.11):
            A_temp = A_temp + 0.02
            
            angle_rad = A_temp * np.sin(self.climbing_omega_spatial * i + self.climbing_phi)
            self.control_angle[2 * i] = angle_rad 
            angle_rad = A_temp * np.cos(self.climbing_omega_spatial * i + self.climbing_phi)
            self.control_angle[2 * i + 1] = angle_rad

            angle_rad = A_temp * np.sin(self.climbing_omega_spatial * (7 - i) + self.climbing_phi)
            self.control_angle[2 * (7 - i)] = angle_rad 
            angle_rad = A_temp * np.cos(self.climbing_omega_spatial * (7 - i) + self.climbing_phi)
            self.control_angle[2 * (7 - i) + 1] = angle_rad

            self.send_control_packet()

        A_temp = 0
        i = 3
        while A_temp < (self.climbing_A_yaw - 0.11):
            A_temp = A_temp + 0.02
            
            angle_rad = A_temp * np.sin(self.climbing_omega_spatial * i + self.climbing_phi)
            self.control_angle[2 * i] = angle_rad 
            angle_rad = A_temp * np.cos(self.climbing_omega_spatial * i + self.climbing_phi)
            self.control_angle[2 * i + 1] = angle_rad

            angle_rad = A_temp * np.sin(self.climbing_omega_spatial * (7 - i) + self.climbing_phi)
            self.control_angle[2 * (7 - i)] = angle_rad 
            angle_rad = A_temp * np.cos(self.climbing_omega_spatial * (7 - i) + self.climbing_phi)
            self.control_angle[2 * (7 - i) + 1] = angle_rad
            
            self.send_control_packet()

        # A_temp = 0
        # while A_temp < (self.climbing_A_yaw - 0.11):
        #     A_temp = A_temp + 0.02
        #     for i in range(8):
        #         angle_rad = A_temp * np.sin(self.climbing_omega_spatial * i + self.climbing_phi)
        #         self.control_angle[2 * i] = angle_rad 
        #         angle_rad = A_temp * np.cos(self.climbing_omega_spatial * i + self.climbing_phi)
        #         self.control_angle[2 * i + 1] = angle_rad
        #     self.send_control_packet()
            # time.sleep(0.1)

    def twining_loosen(self):
        self.climbing_A_pitch = self.climbing_A_pitch + 0.02
        self.climbing_A_yaw = self.climbing_A_yaw + 0.02
        print("climbing_A_yaw: %s" %(self.climbing_A_yaw))
        print("climbing_A_pitch: %s" %(self.climbing_A_pitch))

    def twining_tighten(self):
        self.climbing_A_pitch = self.climbing_A_pitch - 0.02
        self.climbing_A_yaw = self.climbing_A_yaw - 0.02
        print("climbing_A_yaw: %s" %(self.climbing_A_yaw))
        print("climbing_A_pitch: %s" %(self.climbing_A_pitch))

if __name__ == '__main__':
    snake = Climb()
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
                    snake.period = snake.dt_climbing
                    snake.motion_start = 1
                    
                    snake.climbing_curve()
                    print('Start motion')
        if ch == 'e':
            if snake.actuator_enable == 1:
                if snake.motion_start == 1:
                    snake.motion_start = 0
                    print('Pause motion')
        if ch == 'r':  #reset to initial state
            if snake.actuator_enable == 1 and snake.motion_start == 0:
                snake.RS.set_initial_state()
                print('Reset')
        if ch == 't':
            if snake.actuator_enable == 1:
                snake.actuator_enable = 0
                print('Disable actuator')
        if ch == 'y':
            
            print('Terminate process')
            sys.exit()

       
        if ch == '8':
            if snake.actuator_enable == 1:
                snake.speed_up()
                print('Speed up')
        if ch == '2':
            if snake.actuator_enable == 1:
                snake.speed_down()
                print('Speed down')

        if ch == '4':
            if snake.actuator_enable == 1:
                snake.up()
                print('down')
        if ch == '6':
            if snake.actuator_enable == 1:
                snake.down()
                print('up')

        if ch == 'p':  # read position(unit: degree) and load
            if snake.actuator_enable == 1:
                snake.get_position_effort()
                snake.control_angle = snake.read_angle
                print('angle:')
                print(snake.read_angle * 180 / np.pi)
                print('load:')
                print(snake.read_effort)

        if ch == 'l':
            if snake.actuator_enable == 1:
                snake.pipeline_form()
                print('maintain climing form')

        if ch == 'm':
            if snake.actuator_enable == 1:
                snake.pipeline_initial()
                print('initial climing form')


        if ch == 'j':
            if snake.actuator_enable == 1:
                snake.twining_loosen()

        if ch == 'k':
            if snake.actuator_enable == 1:
                snake.twining_tighten()

        ch = ''