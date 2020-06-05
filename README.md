# PeTRA Core

always build and source: 

~/[ws]: $ colcon build

$ source /opt/ros/dashing/setup.bash 

$ source ~/[ws]/install/setup.bash



# How to launch:

In first shell:

$ ros2 launch petra_core petra_launch.py

For user input in second shell:

$ ros2 run petra_drivers Keyboard



For debugging info from each node:

$ rqt



# run single nodes

If you want to run single Nodes in seperate shells:

$ ros2 run petra_drivers Keyboard

$ ros2 run petra_drivers Screen

$ ros2 run petra_drivers RobotDummy

$ ros2 run petra_services Communication

$ ros2 run petra_services Navigation2Dummy

$ ros2 run petra_central_control CCU



# ROS1/2 bridge

source /opt/ros/melodic/setup.bash

source /opt/ros/dashing/setup.bash 

export ROS_MASTER_URI=http://localhost:11311

ros2 run ros1_bridge dynamic_bridge



# Neobotix simulation mmo-500

source /opt/ros/dashing/setup.bash

source ~/neobotix_ws/devel/setup.bash

roslaunch neo_simulation simulation.launch

roslaunch neo_simulation mmo_500_amcl.launch

roslaunch neo_simulation mmo_500_move_base.launch