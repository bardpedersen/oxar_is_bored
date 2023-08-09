#!/usr/bin/expect

# Function to perform SSH using expect
proc ssh_login_and_shutdown {username ip password} {
    spawn ssh $username@$ip
    expect {
        "yes/no" {
            send "yes\r"
            exp_continue
        }
        "password:" {
            send "$password\r"
        }
    }
    
    sleep 2
    expect "\r"
    send "sudo shutdown now\r"
    expect "password for $username:"
    send "$password\r"
    
    sleep 2
    interact
}


# Call the function with provided arguments
ssh_login_and_shutdown "noronn" "192.168.0.52" "noronn"
ssh_login_and_shutdown "noronn" "192.168.0.51" "noronn"
ssh_login_and_shutdown "thorvald" "192.168.0.50" "visitNorway"
