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
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "keyboard_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Int32MultiArray>("joint_angles", 10);

    std_msgs::Int32MultiArray joint_angles;
    joint_angles.data.resize(2);
    joint_angles.data[0] = 0;
    joint_angles.data[1] = 0;

    std::cout << "Use 'q' and 'a' to increase/decrease the first int (0-180)." << std::endl;
    std::cout << "Use 'w' and 's' to increase/decrease the second int (0-180)." << std::endl;
    std::cout << "Press 'x' to exit." << std::endl;

    while (ros::ok())
    {
        int c = getch();

        if (c == 'q')
        {
            if (joint_angles.data[0] < 180)
                joint_angles.data[0]++;
        }
        else if (c == 'a')
        {
            if (joint_angles.data[0] > 0)
                joint_angles.data[0]--;
        }
        else if (c == 'w')
        {
            if (joint_angles.data[1] < 180)
                joint_angles.data[1]++;
        }
        else if (c == 's')
        {
            if (joint_angles.data[1] > 0)
                joint_angles.data[1]--;
        }
        else if (c == 'x')
        {
            break;
        }

        pub.publish(joint_angles);
    }

    return 0;
}
