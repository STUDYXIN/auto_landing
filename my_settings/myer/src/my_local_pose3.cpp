#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>

ros::Publisher multi_pose_pub;
ros::Publisher multi_speed_pub;
ros::Publisher odom_pub;
std::string target_frame, child_frame;
geometry_msgs::PoseStamped multi_local_pose;
geometry_msgs::Vector3Stamped multi_speed;

void modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr& msg) {
    int index = std::find(msg->name.begin(), msg->name.end(), "iris_0") - msg->name.begin();
    nav_msgs::Odometry odom;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";
    odom.pose.pose = msg->pose[index];
    odom.twist.twist = msg->twist[index];
    odom_pub.publish(odom);

    multi_local_pose.header.stamp = ros::Time::now();
    multi_local_pose.header.frame_id = "map";
    multi_local_pose.pose = msg->pose[index];
    multi_speed.header.stamp = ros::Time::now();
    multi_speed.header.frame_id = "map";
    multi_speed.vector = msg->twist[index].linear;
    multi_pose_pub.publish(multi_local_pose);
    multi_speed_pub.publish(multi_speed);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gazebo_to_mavros");
    ros::NodeHandle nh("~");
    multi_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);
    multi_speed_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/mavros/vision_speed/speed", 1);
    ros::Subscriber sub_model_states = nh.subscribe("/gazebo/model_states", 1, modelStatesCallback);
    // odom_pub = nh.advertise<nav_msgs::Odometry>("/mavros/local_position/odom", 1);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/mavros/local_position/odom", 1);


    ros::spin();
    return 0;
}