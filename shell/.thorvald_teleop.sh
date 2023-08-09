#!/bin/bash

#/home/thorvald/Desktop/.thorvald_teleop.sh

sleep 10
aplay /usr/lib/libreoffice/share/gallery/sounds/space.wav

# Function to run ROS launch file in a new gnome terminal
function run_ros_launch_in_xterm() {
  source $HOME/.bashrc
  source /opt/ros/kinetic/setup.bash
  source /opt/ros/kinetic/setup.bash 
  source /home/thorvald/temp_ws/devel/setup.bash
  gnome-terminal -- bash -c "roslaunch $1; exec bash" &
  sleep 5
}

# Run ROS launch files in new gnome terminals
# These files need to be launched before the arms are launched
run_ros_launch_in_xterm "temp_launch thorvald_bringup_remote.launch"
run_ros_launch_in_xterm "thorvald_tunnel_localization aruco_localization.launch"

# Function to check if network is up
check_network() {
  ping -c 1 192.168.0.1 > /dev/null 2>&1
}

# Wait for network to come up
while ! check_network; do
  sleep 5
done

# Function to run ROS launch file in a new gnome terminal
# This file needs to be launched after the network is up
gnome-terminal -- bash -c "source $HOME/.bashrc; roslaunch oxar_is_bored teleop_arm_and_wheels.launch; exec bash" &


