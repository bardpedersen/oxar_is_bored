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

    if (ch == 27 && getchar() == 91)  // Check for escape sequence
    {
        ch = getchar();
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

    std::cout << "Use arrow keys to increase/decrease the joint angles (0-180)." << std::endl;
    std::cout << "Press 'x' to exit." << std::endl;

    while (ros::ok())
    {
        int c = getch();

        switch (c)
        {
            case 65:  // Up arrow key
                if (joint_angles.data[1] < 180)
                    joint_angles.data[1]++;
                break;
            case 66:  // Down arrow key
                if (joint_angles.data[1] > 0)
                    joint_angles.data[1]--;
                break;
            case 67:  // Right arrow key
                if (joint_angles.data[0] < 180)
                    joint_angles.data[0]++;
                break;
            case 68:  // Left arrow key
                if (joint_angles.data[0] > 0)
                    joint_angles.data[0]--;
                break;
            case 'x':
                return 0;
        }

        pub.publish(joint_angles);
    }

    return 0;
}
