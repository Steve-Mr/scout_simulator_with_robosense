import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped

def main():
    rospy.init_node('tf_calculation_node')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # 假设已知的信息
    tag_pose = PoseStamped()  # 在 map 坐标系中的 tag 位姿
    tag_pose.header.frame_id = 'map'
    tag_pose.pose.position.x = 1.0
    tag_pose.pose.position.y = 2.0
    tag_pose.pose.position.z = 0.0
    tag_pose.pose.orientation.w = 1.0

    # 假设已知的 camera 到 tag 的 TF 转换
    camera_to_tag = TransformStamped()
    camera_to_tag.header.frame_id = 'camera'
    camera_to_tag.child_frame_id = 'tag'
    camera_to_tag.transform.translation.x = 0.1
    camera_to_tag.transform.translation.y = 0.2
    camera_to_tag.transform.translation.z = 0.0
    camera_to_tag.transform.rotation.w = 1.0

    # 获取 camera 到 base_link 的 TF 转换
    try:
        camera_to_base_link = tf_buffer.lookup_transform('camera', 'base_link', rospy.Time(0), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Unable to get camera to base_link transform")
        return

    # 计算 base_link 在 map 坐标系中的位姿
    base_link_to_camera = tf2_geometry_msgs.do_transform_pose(tag_pose, camera_to_tag)
    base_link_to_camera.header.frame_id = 'base_link'
    base_link_to_map = tf2_geometry_msgs.do_transform_pose(base_link_to_camera, camera_to_base_link)

    rospy.loginfo("Base link pose in map frame:")
    rospy.loginfo(base_link_to_map.pose)

if __name__ == '__main__':
    main()