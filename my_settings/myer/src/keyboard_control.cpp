#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdio>
#include <ros/console.h>

#define MAX_LINEAR 20.0
#define MAX_ANG_VEL 3.0
#define LINEAR_STEP_SIZE 0.01
#define ANG_VEL_STEP_SIZE 0.01

bool cmd_vel_mask = false;
bool ctrl_leader = false;
std::string msg2leader = "Sending message to the leader";
std::string msg2all = "Sending message to all";
std::string e = "failed";

int kfd = 0;
struct termios oldt, newt;

void init_keyboard_input()
{
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
}

char getKey()
{
    struct termios old_settings, new_settings;
    tcgetattr(STDIN_FILENO, &old_settings);
    new_settings = old_settings;
    new_settings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

    char key = '\0';
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);
    timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;
    select(STDIN_FILENO + 1, &rfds, NULL, NULL, &timeout);

    if (FD_ISSET(STDIN_FILENO, &rfds))
    {
        key = getchar();
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);
    fcntl(STDIN_FILENO, F_SETFL, 0);
    return key;
}
void print_msg()
{
    if (ctrl_leader)
    {
        ROS_INFO("%s", msg2leader.c_str());
    }
    else
    {
        ROS_INFO("%s", msg2all.c_str());
    }
}

int main(int argc, char **argv)
{
     struct termios settings;
    tcgetattr(STDIN_FILENO, &settings);
    std::vector<std::string> formation_configs{"stop controlling"};
    std_msgs::String cmd;
    geometry_msgs::Twist twist;
    ros::init(argc, argv, "iris_multirotor_keyboard_control");

    std::vector<ros::Publisher> multi_cmd_vel_flu_pub(1);
    std::vector<ros::Publisher> multi_cmd_pub(1);
    for (int i = 0; i < 1; ++i) {
        multi_cmd_vel_flu_pub[i] = ros::NodeHandle().advertise<geometry_msgs::Twist>("/xtdrone/iris_" + std::to_string(i) + "/cmd_vel_flu", 1);
        multi_cmd_pub[i] = ros::NodeHandle().advertise<std_msgs::String>("/xtdrone/iris_" + std::to_string(i) + "/cmd", 3);
    }
    ros::Publisher leader_cmd_vel_flu_pub = ros::NodeHandle().advertise<geometry_msgs::Twist>("/xtdrone/leader/cmd_vel_flu", 1);
    ros::Publisher leader_cmd_pub = ros::NodeHandle().advertise<std_msgs::String>("/xtdrone/leader/cmd", 1);

    float forward = 0.3;
    float leftward = 0.0;
    float upward = 0.6;
    float angular = 0.0;

    print_msg();
    init_keyboard_input();

    while (ros::ok())
    {
        char c = getKey();
        switch (c)
        {
        case 'w':
            forward += LINEAR_STEP_SIZE;
            print_msg();
            printf("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f\n", forward, leftward, upward, angular);
            break;
        case 'x':
            forward -= LINEAR_STEP_SIZE;
            print_msg();
            printf("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f\n", forward, leftward, upward, angular);
            break;
        case 'a':
            leftward += LINEAR_STEP_SIZE;
            print_msg();
            printf("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f\n", forward, leftward, upward, angular);
            break;
        case 'd':
            leftward -= LINEAR_STEP_SIZE;
            print_msg();
            printf("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f\n", forward, leftward, upward, angular);
            break;
        case 'i':
            upward = upward + LINEAR_STEP_SIZE;
            print_msg();
            printf("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f\n", forward, leftward, upward, angular);
            break;
        case ',':
            upward = upward - LINEAR_STEP_SIZE;
            print_msg();
            printf("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f\n", forward, leftward, upward, angular);
            break;
        case 'j':
            angular = angular + ANG_VEL_STEP_SIZE;
            print_msg();
            printf("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f\n", forward, leftward, upward, angular);
            break;
        case 'l':
            angular = angular - ANG_VEL_STEP_SIZE;
            print_msg();
            printf("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f\n", forward, leftward, upward, angular);
            break;
        case 'r':
            cmd.data = "AUTO.RTL";
            print_msg();
            printf("Returning home\n");
            break;
        case 't':
            cmd.data = "ARM";
            print_msg();
            printf("Arming\n");
            break;
        case 'y':
            cmd.data = "DISARM";
            print_msg();
            printf("Disarming\n");
            break;
        case 'v':
            cmd.data = "AUTO.TAKEOFF";
            print_msg();
            printf("Takeoff mode is disenabled now\n");
            break;
        case 'b':
            cmd.data = "OFFBOARD";
            print_msg();
            printf("Offboard\n");
            break;
        case 'n':
            cmd.data = "AUTO.LAND";
            print_msg();
            printf("Landing\n");
            break;
        case 'g':
            ctrl_leader = !ctrl_leader;
            print_msg();
            break;
        case 'k':
        case 's':
            cmd_vel_mask = false;
            forward = 0.0;
            leftward = 0.0;
            upward = 0.0;
            angular = 0.0;
            cmd.data = "HOVER";
            print_msg();
            printf("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f ", forward, leftward, upward, angular);
            break;
        default:
            for (int i = 0; i < 10; ++i)
            {
                if (c == std::to_string(i)[0])
                {
                    cmd.data = "stop controlling";
                    print_msg();
                    ROS_INFO_STREAM(cmd);
                    cmd_vel_mask = true;
                }
            }
            if (c == 0x03)
            {
                break;
            }
        }
        if (forward > MAX_LINEAR)
            forward = MAX_LINEAR;
        else if (forward < -MAX_LINEAR)
            forward = -MAX_LINEAR;

        if (leftward > MAX_LINEAR)
            leftward = MAX_LINEAR;
        else if (leftward < -MAX_LINEAR)
            leftward = -MAX_LINEAR;

        if (upward > MAX_LINEAR)
            upward = MAX_LINEAR;
        else if (upward < -MAX_LINEAR)
            upward = -MAX_LINEAR;

        if (angular > MAX_ANG_VEL)
            angular = MAX_ANG_VEL;
        else if (angular < -MAX_ANG_VEL)
            angular = -MAX_ANG_VEL;

        // Set Twist message values
        twist.linear.x = forward;
        twist.linear.y = leftward;
        twist.linear.z = upward;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = angular;

        for (int i = 0; i < 1; ++i)
        {
            if (ctrl_leader)
            {
                leader_cmd_vel_flu_pub.publish(twist);
                leader_cmd_pub.publish(cmd);
            }
            else
            {
                if (!cmd_vel_mask)
                {
                    multi_cmd_vel_flu_pub[i].publish(twist);
                }
                multi_cmd_pub[i].publish(cmd);
            }
        }
        cmd.data = "";

        ros::spinOnce();
    }

    return 0;
}