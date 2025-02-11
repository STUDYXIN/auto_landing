#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdio>
#include <ros/console.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define MAX_LINEAR 5.0
#define MAX_ANG_VEL 3.0
#define LINEAR_STEP_SIZE 0.01
#define ANG_VEL_STEP_SIZE 0.01

bool cmd_vel_mask = false;
bool ctrl_leader = false;
std::string msg2leader = "Sending message to the leader";
std::string msg2all = "Sending message to all";
std::string e = "failed";
float aprfindtime = 0, lastaprtime = 0, begintofly = 0;
int mymode =0;

int kfd = 0;
struct termios oldt, newt;

geometry_msgs::PoseStamped local_pose;



void local_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& data) {
    local_pose = *data;
}

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

    ros::Publisher flu_pub = ros::NodeHandle().advertise<geometry_msgs::Twist>("/xtdrone/cmd_vel_flu", 10);
    ros::Publisher cmd_pub = ros::NodeHandle().advertise<std_msgs::String>("/xtdrone/cmd", 10);

    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    double Kp = 1.0;
    double land_vel = 0.5;
    double track_height = 2.0;
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, local_pose_callback);

    float forward = 0.0;
    float leftward = 0.0;
    float upward = 0.0;
    float angular = 0.0;

    print_msg();
    init_keyboard_input();
    ros::Rate rate(50);

    while (ros::ok())
    {
        char c = getKey();
        switch (c)
        {
        case 't':
            cmd.data = "ARM";
            print_msg();
            printf("Arming\n");
            break;
        case 'b':
            cmd.data = "OFFBOARD";
            print_msg();
            printf("Offboard\n");
            break;
        case 'a':
            mymode = 1; //起飞找马
            begintofly = aprfindtime;
            break;
        case 'l':
            mymode = 2; 
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

                    case '8':
            forward += LINEAR_STEP_SIZE;
            print_msg();
            printf("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f\n", forward, leftward, upward, angular);
            break;
        case '2':
            forward -= LINEAR_STEP_SIZE;
            print_msg();
            printf("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f\n", forward, leftward, upward, angular);
            break;
        case '4':
            leftward += LINEAR_STEP_SIZE;
            print_msg();
            printf("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f\n", forward, leftward, upward, angular);
            break;
        case '6':
            leftward -= LINEAR_STEP_SIZE;
            print_msg();
            printf("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f\n", forward, leftward, upward, angular);
            break;
        case '7':
            upward = upward + LINEAR_STEP_SIZE;
            print_msg();
            printf("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f\n", forward, leftward, upward, angular);
            break;
        case '9':
            upward = upward - LINEAR_STEP_SIZE;
            print_msg();
            printf("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f\n", forward, leftward, upward, angular);
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
        //**************************************************************
        geometry_msgs::TransformStamped tfStamped;
        try {
            tfStamped = tfBuffer.lookupTransform("tag_0", "base_link", ros::Time(0));

        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
        //forward = Kp * (tfStamped.transform.translation.x - local_pose.pose.position.x);
        //leftward = Kp * (tfStamped.transform.translation.y - local_pose.pose.position.y);
        // Set Twist message values
        //ROS_INFO("Header stamp: %f", tfStamped.header.stamp.toSec());
        aprfindtime = tfStamped.header.stamp.toSec();//判断是否找到april码的依据
        
        if(mymode == 1)
        {
            if(aprfindtime - begintofly< 1e-4) //从0开始起飞阶段
            {
                forward = 0.3;
                upward = 0.6;
            }
            else
            {
                forward = Kp * (tfStamped.transform.translation.x);
                leftward = Kp * (tfStamped.transform.translation.y);
                upward = Kp * (-tfStamped.transform.translation.z + track_height);
                //printf("tf: %.2f\t lo: %.2f\n", tfStamped.transform.translation.z, local_pose.pose.position.z);
            }
        }

        else if (mymode == 2)
        {
            forward = Kp * (tfStamped.transform.translation.x);
            leftward = Kp * (tfStamped.transform.translation.y);
            upward = Kp * (-tfStamped.transform.translation.z);
            if(aprfindtime - begintofly< 1e-4)
            {
                ros::Duration(1).sleep();
                forward = 0.0;
                leftward = 0.0;
                upward = 0.0;
                angular = 0.0;
                mymode = 0;
            }
        }
        lastaprtime = aprfindtime;

        //*************************************************************
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
        twist.linear.x = forward;
        twist.linear.y = leftward;
        twist.linear.z = upward;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = angular;

        flu_pub.publish(twist);
        cmd_pub.publish(cmd);

        ros::spinOnce();

        
    }
    tcsetattr(kfd, TCSANOW, &oldt);

    return 0;
}