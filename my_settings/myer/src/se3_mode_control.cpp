#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::Publisher tag_pose_pub;

int main(int argc, char** argv) {
    ros::init(argc, argv, "tag_0_pose");
    ros::NodeHandle nh("~");
    tag_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/tag_0_pose", 1);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    while (ros::ok())
    {
        //**************************************************************
        geometry_msgs::TransformStamped tfStamped;
        try {
            tfStamped = tfBuffer.lookupTransform("map", "tag_1", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
        // forward = Kp * (tfStamped.transform.translation.x - local_pose.pose.position.x);
        // leftward = Kp * (tfStamped.transform.translation.y - local_pose.pose.position.y);
        //  Set Twist message values
        // ROS_INFO("Header stamp: %f", tfStamped.header.stamp.toSec());
        geometry_msgs::PoseStamped tag_pose;
        tag_pose.header.frame_id = "map";
        tag_pose.pose.position.x = tfStamped.transform.translation.x;
        tag_pose.pose.position.y = tfStamped.transform.translation.y;
        tag_pose.pose.position.z = tfStamped.transform.translation.z;
        tag_pose.pose.orientation= tfStamped.transform.rotation;
        tag_pose_pub.publish(tag_pose);
        //ROS_INFO("desire posit: %f %f %f", tag_pose.pose.position.x, tag_pose.pose.position.x, tag_pose.pose.position.x);
        ros::spinOnce();
        
    }

    return 0;
}