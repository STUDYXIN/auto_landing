#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::Publisher pose_pub;
ros::Publisher velocity_pub;
geometry_msgs::Twist twist;
geometry_msgs::PoseStamped pose_stamped_imu;
geometry_msgs::Vector3Stamped velocity_stamped_imu;

float aprfindtime = 0, lastaprtime = 0, begintofly = 0;

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    pose_stamped_imu.header = msg->header;
    pose_stamped_imu.header.frame_id = "map";
    pose_stamped_imu.pose.orientation = msg->orientation;
    // pose_pub.publish(pose_stamped_imu);
    velocity_stamped_imu.header = msg->header;
    velocity_stamped_imu.header.frame_id = "map";
    velocity_stamped_imu.vector = msg->linear_acceleration;
    // velocity_pub.publish(pose_stamped_imu);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_data_subscriber");

    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
    velocity_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/mavros/vision_speed/speed", 10);

    ros::Subscriber imu_sub = nh.subscribe("/mavros/imu/data", 10, imu_callback);
    ROS_INFO("Get iris_0 groundtruth pose");

    ros::Rate rate(10.0);

    while (ros::ok())
    {
        // Lookup the transform from tag_0 to base_link
        geometry_msgs::TransformStamped tfStamped;
        geometry_msgs::TransformStamped tfStamped2;
        try
        {
            //tfStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
            tfStamped = tfBuffer.lookupTransform("tag_0", "base_link", ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            //ROS_WARN("%s", ex.what());
            //ros::Duration(1.0).sleep();
            //continue;
        }

        // Convert transform to pose and vector

        aprfindtime = tfStamped2.header.stamp.toSec();//判断是否找到april码的依据

        geometry_msgs::PoseStamped pose_stamped_tf;
        pose_stamped_tf.header.stamp = tfStamped.header.stamp;
        pose_stamped_tf.header.frame_id = "map";
        pose_stamped_tf.pose.position.x = tfStamped.transform.translation.x + 0.3;
        pose_stamped_tf.pose.position.y = tfStamped.transform.translation.y;
        pose_stamped_tf.pose.position.z = tfStamped.transform.translation.z + 1.75;
        pose_stamped_tf.pose.orientation = tfStamped.transform.rotation;
        //pose_pub.publish(pose_stamped);

        geometry_msgs::Vector3Stamped vector_stamped_tf;
        vector_stamped_tf.header.stamp = tfStamped.header.stamp;
        vector_stamped_tf.header.frame_id = "map";
        vector_stamped_tf.vector.x = tfStamped.transform.translation.x;
        vector_stamped_tf.vector.y = tfStamped.transform.translation.y;
        vector_stamped_tf.vector.z = tfStamped.transform.translation.z;
        // velocity_pub.publish(vector_stamped);

        if (aprfindtime - begintofly < 1e4) // 从0开始起飞阶段
        {
            //pose_pub.publish(pose_stamped_imu);
            //velocity_pub.publish(velocity_stamped_imu);
            //printf("x: %.2f\t y: %.2f\t z: %.2f\n", pose_stamped_tf.pose.position.x, pose_stamped_tf.pose.position.y, pose_stamped_tf.pose.position.z);
        }
        else
        {
            // nh.setParam("mavros/global_position/frame_id", "tag_0");
            pose_pub.publish(pose_stamped_tf);
            velocity_pub.publish(vector_stamped_tf);
            printf("tf: %.2f\n", aprfindtime);
        }

        lastaprtime = aprfindtime;

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}