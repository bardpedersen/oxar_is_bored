#!/bin/bash

#/home/noronn/Desktop/.arm2_launch.sh

# Function to check if network is up
check_network() {
  ping -c 1 192.168.0.1 > /dev/null 2>&1
}

# Wait for network to come up
while ! check_network; do
  sleep 5
done


# Function to run ROS launch file in a new xterm terminal
function run_ros_launch_in_xterm() {
  gnome-terminal -- bash -ic "source $HOME/.bashrc; roslaunch $1; exec bash" &
  sleep 5
}

# Run ROS launch files in new xterm terminals
run_ros_launch_in_xterm "oxar_is_bored distance_sensors.launch"
run_ros_launch_in_xterm "harvester harvester_arm2.launch" 
run_ros_launch_in_xterm "oxar_is_bored end_effector2.launch"
run_ros_launch_in_xterm "realsense2_camera rs_camera.launch camera:=camera_l1 serial_no:=746112060879 filters:=pointcloud initial_reset:=true depth_fps:=15 color_fps:=15"
run_ros_launch_in_xterm "realsense2_camera rs_camera.launch camera:=camera_l2 serial_no:=825312073170 filters:=pointcloud initial_reset:=true depth_fps:=15 color_fps:=15"
run_ros_launch_in_xterm "oxar_is_bored bag_record_left.launch"