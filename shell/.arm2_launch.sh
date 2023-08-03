#!/bin/bash

#/home/noronn/Desktop/.arm2_launch.sh

check_network() {
  ping -c 1 192.168.0.1 > /dev/null 2>&1
}

while ! check_network; do
  sleep 5
done


# Function to run ROS launch file in a new xterm terminal
function run_ros_launch_in_xterm() {
  gnome-terminal -- bash -ic "source $HOME/.bashrc; roslaunch $1; exec bash" &
  sleep 5
}

run_ros_launch_in_xterm "harvester harvester_arm2.launch" 
run_ros_launch_in_xterm "oxar_is_bored end_effector2.launch"
run_ros_launch_in_xterm "realsense2_camera rs_camera.launch camera:=cam_4 serial_no:=746112060879 filters:=pointcloud"
run_ros_launch_in_xterm "realsense2_camera rs_camera.launch camera:=cam_3 serial_no:=825312073170 filters:=pointcloud"