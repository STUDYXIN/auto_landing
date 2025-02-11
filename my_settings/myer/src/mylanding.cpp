#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

geometry_msgs::PoseStamped local_pose;

void local_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& data) {
    local_pose = *data;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "precision_landing");
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::Twist cmd_vel_enu;
    double Kp = 1.0;
    double land_vel = 0.5;
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/iris_0/mavros/local_position/pose", 1, local_pose_callback);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/xtdrone/iris_0/cmd_vel_enu", 1);
    ros::Rate rate(50);

    while (ros::ok()) {
        geometry_msgs::TransformStamped tfStamped;
        try {
            tfStamped = tfBuffer.lookupTransform("map", "tag_0", ros::Time(0));

        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            continue;
        }
        cmd_vel_enu.linear.x = Kp * (tfStamped.transform.translation.x - local_pose.pose.position.x);
        cmd_vel_enu.linear.y = Kp * (tfStamped.transform.translation.y - local_pose.pose.position.y);
        cmd_vel_enu.linear.z = -land_vel;
        cmd_vel_pub.publish(cmd_vel_enu);
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}