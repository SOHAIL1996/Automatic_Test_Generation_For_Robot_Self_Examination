#!/bin/bash

sudo pkill gazebo
sudo pkill gzserver
sudo pkill gzclient
sudo pkill roscore

python3 src/RSG.py
# rosrun rviz rviz


# roslaunch mas_hsr_move_arm_action move_arm.launch 
# roslaunch mas_hsr_move_base_action move_base.launch 
# rqt
# rosrun moveit_commander moveit_commander_cmdline.py
# rosrun mdr_move_base_action move_base_action_client_test "{kitchen: ,}"

# echo $PATH
# export PATH=$PATH:directory
