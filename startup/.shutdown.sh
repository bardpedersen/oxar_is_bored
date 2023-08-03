#!/usr/bin/env expect 

spawn ssh thorvald@192.168.0.50 
expect "password:"
send "visitNorway\r"
sleep 2
expect "\r"
send "sudo shutdown -h 0\r"
expect "password for thorvald:"
send "visitNorway\r"

sleep 2
spawn ssh noronn@192.168.0.51
expect "password:"
send "noronn\r"
sleep 2
expect "\r"
send "sudo shutdown -h 0\r"
expect "password for noronn:"
send "noronn\r"

sleep 2
spawn ssh noronn@192.168.0.52 
expect "password:"
send "noronn\r"
sleep 2
expect "\r"
send "sudo shutdown -h 0\r"
expect "password for noronn:"
send "noronn\r"
expect eof
