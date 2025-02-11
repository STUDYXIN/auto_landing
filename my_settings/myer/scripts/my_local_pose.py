#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
import sys
from gazebo_msgs.msg import ModelStates

multi_pose_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
multi_speed_pub = rospy.Publisher('/mavros/vision_speed/speed', Vector3Stamped, queue_size=1)
multi_local_pose = PoseStamped()
multi_speed = Vector3Stamped()

def gazebo_model_state_callback(msg):
    id = msg.name.index('iris_0')
    multi_local_pose.header.stamp = rospy.Time().now()
    multi_local_pose.header.frame_id = 'map'
    multi_local_pose.pose = msg.pose[id]
    multi_speed.header.stamp = rospy.Time().now()
    multi_speed.header.frame_id = 'map'
    multi_speed.vector = msg.twist[id]
    multi_pose_pub.publish(multi_local_pose)
    multi_speed_pub.publish(multi_speed)

if __name__ == '__main__':
    rospy.init_node('get_pose_groundtruth')
    gazebo_model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_model_state_callback,queue_size=1)
    print("Get iris_0 groundtruth pose")
    rospy.spin()