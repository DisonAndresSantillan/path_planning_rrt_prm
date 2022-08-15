#!/bin/bash
clear
echo "roscore iniciado"
gnome-terminal -- sh -c "roscore; bash" &
sleep 10
echo "roscore iniciado"
# Inicio gmapping
gnome-terminal -- sh -c "cd; cd catkin_ws/src;roslaunch path_planning_rrt_prm/gmapping/launch_gmapping.launch
; bash" &
sleep 10
echo "Entorno en ROS Gazebo iniciado"
# inicio slam
gnome-terminal -- sh -c "cd; cd catkin_ws/src;roslaunch path_planning_rrt_prm/gmapping/launch_slam.launch
; bash" &
sleep 5
echo "Entorno en Rviz iniciado"



