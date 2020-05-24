# rl_snake_robot
build a ros/gazebo simulator for snake_like robot, test deep reinforcement learning for climbing pipeline
# environments
ROS+ros_control+gazebo, Anaconda + tensorflow
# firstly, load the model of snake-like robot
1. extract snake_climb
2. cd snake_climb
3. source ./devel/setup.bash
4. cd src
5. roslaunch snake_climb/launch/snake_world.launch
# secondly, load the PID controller for the joint angle of snake-like robot
1. cd snake_climb
2. source ./devel/setup.bash
3. cd src
4. roslaunch snake_climb/snake_control/launch/snake_control.launch
# test the deep reinforcement learning 
all related code are located in the folder 
snake_climb/src/snake_climb/src
