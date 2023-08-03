#!/bin/bash

#/home/thorvald/Desktop/.thorvald_teleop.sh

sleep 30
aplay /usr/lib/libreoffice/share/gallery/sounds/space.wav

function run_ros_launch_in_xterm() {
  source $HOME/.bashrc
  source /opt/ros/kinetic/setup.bash
  source /opt/ros/kinetic/setup.bash 
  source /home/thorvald/temp_ws/devel/setup.bash
  gnome-terminal -- bash -c "roslaunch $1; exec bash" &
  sleep 5
}

run_ros_launch_in_xterm "temp_launch thorvald_bringup_remote.launch"
run_ros_launch_in_xterm "thorvald_tunnel_localization aruco_localization.launch"
