# Setup simulation

roscore

roslaunch miarobot_description miarobot_gazebo.launch

# Setup of robot

roscore

roslaunch Mapping mapping.launch

roslaunch miarobot_description miarobot_gazebo.launch #if simulating, do note that u must put it at the exact postion thats in the map before running the script

rosrun Mapping mapping.py

rosrun Mapping pid_controller.py

## Miarobot_Description package

It contains the world and the robot with the launch file to run gazebo with both of them

# Mapping package
