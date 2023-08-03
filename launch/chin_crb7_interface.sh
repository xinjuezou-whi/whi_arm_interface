#!/bin/bash
cd /home/whi/catkin_workspace/
source /opt/ros/melodic/setup.bash
source /home/whi/catkin_workspace/devel/setup.bash
echo "launching application, please wait..."
roslaunch whi_arm_interface whi_arm_hardware_interface.launch arm_model:=crb7
