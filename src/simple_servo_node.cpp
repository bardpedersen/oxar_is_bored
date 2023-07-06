#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

int getch()
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    ch = getchar();

    // Handle escape sequences for arrow keys
    if (ch == 27)
    {
        ch = getchar();
        if (ch == 91)
        {
            ch = getchar();
            switch (ch)
            {
            case 65: // Up arrow key
                ch = 'w';
                break;
            case 66: // Down arrow key
                ch = 's';
                break;
            case 67: // Right arrow key
                ch = 'q';
                break;
            case 68: // Left arrow key
                ch = 'a';
                break;
            }
        }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Int32MultiArray>("joint_angles", 20);

    std_msgs::Int32MultiArray joint_angles;
    joint_angles.data.resize(2);
    joint_angles.data[0] = 90;
    joint_angles.data[1] = 90;

    std::cout << "Use the arrow keys to increase/decrease the joint angles (0-180)." << std::endl;
    std::cout << "Press 'x' to exit." << std::endl;

    while (ros::ok())
    {
        int c = getch();

        if (c == 'w')
        {
            if (joint_angles.data[0] < 180)
                joint_angles.data[0] += 10;
        }
        else if (c == 's')
        {
            if (joint_angles.data[0] > 0)
                joint_angles.data[0] -= 10;
        }
        else if (c == 'q')
        {
            if (joint_angles.data[1] < 180)
                joint_angles.data[1] += 10;
        }
        else if (c == 'a')
        {
            if (joint_angles.data[1] > 0)
                joint_angles.data[1] -= 10;
        }
        else if (c == 'x')
        {
            break;
        }

        pub.publish(joint_angles);
    }

    return 0;
}
