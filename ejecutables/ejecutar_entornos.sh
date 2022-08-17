#!/bin/bash
clear

gnome-terminal -- sh -c "roscore; bash" &
sleep 10
echo "roscore iniciado.........."
# Inicio gmapping
gnome-terminal -- sh -c "cd; cd catkin_ws/src;roslaunch path_planning_rrt_prm/launch/launch_gazebo.launch; bash" &
sleep 10
echo "Entorno en ROS Gazebo iniciado.......... "
echo "roslaunch path_planning_rrt_prm/launch/launch_gazebo.launch"
# inicio slam
gnome-terminal -- sh -c "cd; cd catkin_ws/src;roslaunch path_planning_rrt_prm/launch/launch_rviz.launch; bash" &
sleep 5
echo "Entorno en Rviz iniciado.........."
echo "roslaunch path_planning_rrt_prm/launch/launch_rviz.launch"


