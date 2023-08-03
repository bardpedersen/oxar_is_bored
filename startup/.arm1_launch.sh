#!/bin/bash

#/home/noronn/Desktop/.arm1_launch.sh

sleep 50

# Function to run ROS launch file in a new xterm terminal
function run_ros_launch_in_xterm() {
  gnome-terminal -- bash -ic "source $HOME/.bashrc; roslaunch $1; exec bash" &
  sleep 5
}

run_ros_launch_in_xterm "harvester harvester_arm1.launch" 
run_ros_launch_in_xterm "oxar_is_bored end_effector1.launch"
run_ros_launch_in_xterm "realsense2_camera rs_camera.launch camera:=cam_2 serial_no:=806312060325 filters:=pointcloud"
run_ros_launch_in_xterm "realsense2_camera rs_camera.launch camera:=cam_1 serial_no:=825312071978 filters:=pointcloud"