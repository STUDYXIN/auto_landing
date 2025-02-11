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
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <math.h>
#include <random>

#define MAX_LINEAR 50.0
#define MAX_ANG_VEL 3.0
#define LINEAR_STEP_SIZE 0.01
#define ANG_VEL_STEP_SIZE 0.01

bool cmd_vel_mask = false;
bool ctrl_leader = false;
std::string msg2leader = "Sending message to the leader";
std::string msg2all = "Sending message to all";
std::string e = "failed";
float aprfindtime = 0, lastaprtime = 0, begintofly = 0;
int firstfindar = 0;
int mymode =0, count=0;

float ekf_noise = 0, imu_noise = 0;
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

    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/myertf/basetotag", 10);
    ros::Publisher realpose_pub = nh.advertise<geometry_msgs::PoseStamped>("/realpose", 10);
    ros::Publisher estimate_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/estimate_pose", 10);
    ros::Publisher estimate_noise_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/estimate_noise_pose", 10);
    ros::Publisher fusion_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/fusion_pose", 10);
    ros::Publisher realpath_pub = nh.advertise<nav_msgs::Path>("/realpath", 10);
    ros::Publisher estimate_path_pub = nh.advertise<nav_msgs::Path>("/estimate_path", 10);


    geometry_msgs::Vector3Stamped vector_stamped_tf;

    double Kp = 1.0;
    double land_vel = 0.5;
    double track_height = 2.0;
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, local_pose_callback);

    float forward = 0.0;
    float leftward = 0.0;
    float upward = 0.0;
    float angular = 0.0;

    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, 0.1);
    float ekf_add = 0, ekf_add2=0;

    // 生成均值为0、方差为1的噪声
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
                mymode = 1; // 起飞找马
                begintofly = aprfindtime;
                break;
            case 'm':
                mymode = 2;
                break;
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

            case 'i':
                forward += LINEAR_STEP_SIZE;
                print_msg();
                printf("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f\n", forward, leftward, upward, angular);
                break;
            case 'k':
                forward -= LINEAR_STEP_SIZE;
                print_msg();
                printf("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f\n", forward, leftward, upward, angular);
                break;
            case 'j':
                leftward += LINEAR_STEP_SIZE;
                print_msg();
                printf("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f\n", forward, leftward, upward, angular);
                break;
            case 'l':
                leftward -= LINEAR_STEP_SIZE;
                print_msg();
                printf("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f\n", forward, leftward, upward, angular);
                break;
            case 'o':
                upward = upward + LINEAR_STEP_SIZE;
                print_msg();
                printf("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f\n", forward, leftward, upward, angular);
                break;
            case 'p':
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
            geometry_msgs::TransformStamped tfStamped2;
            geometry_msgs::TransformStamped tfStamped3;
            try
            {
                tfStamped = tfBuffer.lookupTransform("tag_0", "base_link", ros::Time(0));
                tfStamped2 = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
                tfStamped3 = tfBuffer.lookupTransform("tag_0", "base_link", ros::Time(0));
                vector_stamped_tf.header.stamp = tfStamped.header.stamp;
                vector_stamped_tf.header.frame_id = "tag_0";
                vector_stamped_tf.vector.x = tfStamped.transform.translation.x;
                vector_stamped_tf.vector.y = tfStamped.transform.translation.y;
                vector_stamped_tf.vector.z = tfStamped.transform.translation.z;
                velocity_pub.publish(vector_stamped_tf);
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
            }
            // forward = Kp * (tfStamped.transform.translation.x - local_pose.pose.position.x);
            // leftward = Kp * (tfStamped.transform.translation.y - local_pose.pose.position.y);
            //  Set Twist message values
            // ROS_INFO("Header stamp: %f", tfStamped.header.stamp.toSec())
            aprfindtime = tfStamped.header.stamp.toSec(); // 判断是否找到april码的依据

            // 发布真实位姿
            geometry_msgs::PoseStamped realpose;
            realpose.header.frame_id = "tag_ekf";
            realpose.pose.position.x = tfStamped2.transform.translation.x + 1.3;
            realpose.pose.position.y = tfStamped2.transform.translation.y;
            realpose.pose.position.z = tfStamped2.transform.translation.z - 1.75;
            realpose.pose.orientation = tfStamped2.transform.rotation;
            realpose_pub.publish(realpose);

            nav_msgs::Path realpath;
            realpath.header.frame_id = "tag_ekf";
            realpath.header.stamp = ros::Time::now();
            realpath.poses.push_back(realpose);
            realpath_pub.publish(realpath);

            // 发布估计位姿
            geometry_msgs::PoseStamped estimate_pose;
            estimate_pose.header.frame_id = "tag_ekf";
            estimate_pose.pose.position.x = tfStamped3.transform.translation.x;
            estimate_pose.pose.position.y = tfStamped3.transform.translation.y;
            estimate_pose.pose.position.z = tfStamped3.transform.translation.z;
            estimate_pose.pose.orientation = tfStamped3.transform.rotation;
            estimate_pose_pub.publish(estimate_pose);

            nav_msgs::Path estimate_path;
            estimate_path.header.frame_id = "tag_ekf";
            estimate_path.header.stamp = ros::Time::now();
            estimate_path.poses.push_back(estimate_pose);
            estimate_path_pub.publish(estimate_path);

            ekf_noise = distribution(generator);
            ekf_add += 0.01;
            geometry_msgs::PoseStamped estimate_noise_pose;
            estimate_noise_pose.header.frame_id = "tag_ekf";
            // estimate_noise_pose.pose.position.x = tfStamped2.transform.translation.x + 0.02*ekf_noise +1.3 + 0.7*exp(ekf_add);
            estimate_noise_pose.pose.position.x = tfStamped2.transform.translation.x + 0.02*ekf_noise + 0.7*exp(ekf_add) + 0.7;
            estimate_noise_pose.pose.position.y = tfStamped2.transform.translation.y + 0.01 *ekf_noise + 0.2*exp(ekf_add);
            estimate_noise_pose.pose.position.z = tfStamped2.transform.translation.z + 0.04 * ekf_noise -1.75;
            estimate_noise_pose.pose.orientation.x = tfStamped2.transform.rotation.x + 0.04*ekf_noise;
            estimate_noise_pose.pose.orientation.y = tfStamped2.transform.rotation.y + 0.01*ekf_noise;
            estimate_noise_pose.pose.orientation.z = tfStamped2.transform.rotation.z + 0.01*ekf_noise;
            estimate_noise_pose.pose.orientation.w = tfStamped2.transform.rotation.w + 0.02*ekf_noise;
            estimate_noise_pose_pub.publish(estimate_noise_pose);

            geometry_msgs::PoseStamped fusion_pose;
            ROS_WARN("%f",aprfindtime - lastaprtime);
            if (aprfindtime - lastaprtime > 1e-4) // 从0开始起飞阶段
            {
                fusion_pose=estimate_pose;
                ekf_add2=0;
            }
            else if(count <=2)
            {
                count++;
            }
            else
            {ekf_add2 += 0.01;
                count =0;
                fusion_pose.header.frame_id = "tag_ekf";
                // estimate_noise_pose.pose.position.x = tfStamped2.transform.translation.x + 0.02*ekf_noise +1.3 + 0.7*exp(ekf_add);
                fusion_pose.pose.position.x = tfStamped2.transform.translation.x + 0.02 * ekf_noise + 0.7 * exp(ekf_add2) + 0.7;
                fusion_pose.pose.position.y = tfStamped2.transform.translation.y + 0.01 * ekf_noise + 0.2 * exp(ekf_add2);
                fusion_pose.pose.position.z = tfStamped2.transform.translation.z + 0.04 * ekf_noise - 1.75;
                fusion_pose.pose.orientation.x = tfStamped2.transform.rotation.x + 0.04 * ekf_noise;
                fusion_pose.pose.orientation.y = tfStamped2.transform.rotation.y + 0.01 * ekf_noise;
                fusion_pose.pose.orientation.z = tfStamped2.transform.rotation.z + 0.01 * ekf_noise;
                fusion_pose.pose.orientation.w = tfStamped2.transform.rotation.w + 0.02 * ekf_noise;
            }
            fusion_pose_pub.publish(fusion_pose);

            if (mymode == 1)
            {
                if (aprfindtime - begintofly < 1e-4) // 从0开始起飞阶段
                {
                    forward = 0.1;
                    upward = 0.35;
                }
                else
                {
                    // 第一次找到apriltag码,建立新的分支，删除原有分支
                    forward = Kp * (tfStamped.transform.translation.x);
                    leftward = Kp * (tfStamped.transform.translation.y);
                    upward = 0.1 * (-tfStamped.transform.translation.z + track_height);
                    // forward = 0;
                    // leftward = 0;
                    // upward = -5.0;
                    printf("x: %.2f\t y: %.2f\t z: %.2f\n", tfStamped.transform.translation.x, tfStamped.transform.translation.y, tfStamped.transform.translation.z);
                }
            }

            else if (mymode == 2)
            {
                forward = Kp * (tfStamped.transform.translation.x);
                leftward = Kp * (tfStamped.transform.translation.y);
                upward = Kp * (-tfStamped.transform.translation.z);
                if (aprfindtime - begintofly < 1e-4)
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