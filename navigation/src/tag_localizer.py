#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

import suppress_warnings


def main():
    pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)

    rospy.init_node('tf_calculation_node')
    suppress_warnings.suppress_TF_REPEATED_DATA()

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # 假设已知的信息
    tag_pose = PoseStamped()  # 在 map 坐标系中的 tag 位姿
    tag_pose.header.frame_id = 'map'
    tag_pose.pose.position.x = 0.06966787598510285
    tag_pose.pose.position.y = 3.545644146622064
    tag_pose.pose.position.z = 0.7671298894104502
    tag_pose.pose.orientation.x = 0.5033900325838786
    tag_pose.pose.orientation.y = 0.5013064471502089
    tag_pose.pose.orientation.z = -0.4990135926362092
    tag_pose.pose.orientation.w = 0.49626178122553405

    # 假设已知的 camera 到 tag 的 TF 转换
    camera_to_tag = TransformStamped()
    camera_to_tag.header.frame_id = 'camera'
    camera_to_tag.child_frame_id = 'tag'
    camera_to_tag.transform.translation.x = 0.1
    camera_to_tag.transform.translation.y = 0.2
    camera_to_tag.transform.translation.z = 0.0
    camera_to_tag.transform.rotation.w = 1.0
    
    # 获取 tag 到 camera 的 TF 转换

    # try:
    #     base_link_to_tag = tf_buffer.lookup_transform('tag01', 'base_link', rospy.Time(0), rospy.Duration(5.0))
    # except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
    #     rospy.logwarn("Unable to get camera to base_link transform")
    #     return

    # 获取 camera 到 base_link 的 TF 转换
    try:
        camera_to_base_link = tf_buffer.lookup_transform('tag01', 'base_link', rospy.Time(0), rospy.Duration(10.0))
        print(camera_to_base_link)
    except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Unable to get camera to base_link transform")
        return

    # 计算 base_link 在 map 坐标系中的位姿
    # base_link_to_camera = tf2_geometry_msgs.do_transform_pose(tag_pose, camera_to_tag)
    # base_link_to_camera.header.frame_id = 'base_link'
    base_link_to_map = tf2_geometry_msgs.do_transform_pose(tag_pose, camera_to_base_link)

    rospy.loginfo("Base link pose in map frame:")
    rospy.loginfo(base_link_to_map)
    
    
    initial_pose_msg = PoseWithCovarianceStamped()
    initial_pose_msg.header.stamp = rospy.Time.now()
    initial_pose_msg.header.frame_id = 'map'  # 设置map坐标系作为参考坐标系
    
    # 设置正确的base_link位姿信息，假设你已经有了正确的位姿值
    initial_pose_msg.pose.pose = base_link_to_map.pose
    # initial_pose_msg.pose.covariance = ' [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]'
    
    print(initial_pose_msg)
    
    pub.publish(initial_pose_msg)
    
    test_pub = rospy.Publisher('testpose', PoseStamped)
    test_pub.publish(base_link_to_map)
    
    print('published')
    
    rospy.spin()

if __name__ == '__main__':
    main()