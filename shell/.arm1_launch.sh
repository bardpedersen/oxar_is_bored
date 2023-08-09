#!/bin/bash

#/home/noronn/Desktop/.arm1_launch.sh

# Function to check if network is up
check_network() {
  ping -c 1 192.168.0.1 > /dev/null 2>&1
}

# Wait for network to come up
while ! check_network; do
  sleep 5
done

# Function to run ROS launch file in a new gnome terminal
function run_ros_launch_in_xterm() {
  gnome-terminal -- bash -ic "source $HOME/.bashrc; roslaunch $1; exec bash" &
  sleep 5
}

# Run ROS launch files in new gnome terminals
run_ros_launch_in_xterm "harvester harvester_arm1.launch" 
run_ros_launch_in_xterm "oxar_is_bored end_effector1.launch"
run_ros_launch_in_xterm "realsense2_camera rs_camera.launch camera:=cam_2 serial_no:=806312060325 filters:=pointcloud"
run_ros_launch_in_xterm "realsense2_camera rs_camera.launch camera:=cam_1 serial_no:=825312071978 filters:=pointcloud"