#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <gazebo_msgs/ModelStates.h>

ros::Publisher multi_pose_pub;
ros::Publisher multi_speed_pub;
geometry_msgs::PoseStamped multi_local_pose;
geometry_msgs::Vector3Stamped multi_speed;

void gazebo_model_state_callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    int id = std::find(msg->name.begin(), msg->name.end(), "iris_0") - msg->name.begin();
    multi_local_pose.header.stamp = ros::Time::now();
    multi_local_pose.header.frame_id = "map";
    multi_local_pose.pose = msg->pose[id];
    multi_speed.header.stamp = ros::Time::now();
    multi_speed.header.frame_id = "map";
    multi_speed.vector = msg->twist[id].linear;
    multi_pose_pub.publish(multi_local_pose);
    multi_speed_pub.publish(multi_speed);

    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "get_pose_groundtruth");
    ros::NodeHandle nh;
    multi_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);
    multi_speed_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/mavros/vision_speed/speed", 1);
    ros::Subscriber gazebo_model_state_sub = nh.subscribe("/gazebo/model_states", 1, gazebo_model_state_callback);
    ROS_INFO("Get iris_0 groundtruth pose");
    ros::spin();
    return 0;
}