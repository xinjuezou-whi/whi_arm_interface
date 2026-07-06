#!/bin/bash
cd /home/nvidia/ros2_ws/
source /home/nvidia/ros2_kilted/install/local_setup.bash
source /home/nvidia/ros2_nav2/install/local_setup.bash
source /home/nvidia/ros2_moveit2/install/local_setup.bash
source /home/nvidia/ros2_ws/install/local_setup.bash
echo "launching application, please wait..."
roslaunch whi_arm_interface launch.py arm:=jaka arm_model:=a5
