#!/usr/bin/env python3

import rospy
import yaml
from tf.msg import tfMessage
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_geometry_msgs


import suppress_warnings

rospy.init_node('tag_logger')
suppress_warnings.suppress_TF_REPEATED_DATA()

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

map_pose = PoseStamped()  # 在 map 坐标系中的 tag 位姿
map_pose.header.frame_id = 'map'
map_pose.pose.position.x = 0
map_pose.pose.position.y = 0
map_pose.pose.position.z = 0
map_pose.pose.orientation.x = 0
map_pose.pose.orientation.y = 0
map_pose.pose.orientation.z = 0
map_pose.pose.orientation.w = 1

def look_for_tag(msg):
    for transform in msg.transforms:
        frame_name = transform.child_frame_id
        if frame_name in tags_dict.keys():
            print(f"Detected new TF frame: {frame_name}")

            # tf2 ver
            try:
                trans = tfBuffer.lookup_transform('map', frame_name, rospy.Time())
                tag_pose = tf2_geometry_msgs.do_transform_pose(map_pose, trans)
                print(trans)
                param_value[tags_dict[frame_name]]['x'] = tag_pose.pose.position.x
                param_value[tags_dict[frame_name]]['y'] = tag_pose.pose.position.y
                param_value[tags_dict[frame_name]]['z'] = tag_pose.pose.position.z
                param_value[tags_dict[frame_name]]['qx'] = tag_pose.pose.orientation.x
                param_value[tags_dict[frame_name]]['qy'] = tag_pose.pose.orientation.y
                param_value[tags_dict[frame_name]]['qz'] = tag_pose.pose.orientation.z
                param_value[tags_dict[frame_name]]['qw'] = tag_pose.pose.orientation.w
                with open(param_dir, 'r') as f:
                    params = yaml.safe_load(f)
                    params['standalone_tags'] = param_value
                with open(param_dir, 'w') as f:
                    yaml.safe_dump(params, f)
                tags_dict.pop(frame_name)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print('ah-oh')

            
# read param
param_dir = rospy.get_param("/tags_dir", '/home/maary/slam/scout_ws/src/navigation/config/tags.yaml')
param_value = rospy.get_param('/apriltag_ros_continuous_node/standalone_tags')

print(param_value)
print(type(param_value))
print(type(param_value[0]))
param_value[1]['name'] = 'tag03'
print(param_value[1])
rospy.set_param('/apriltag_ros_continuous_node/standalone_tags', param_value)

# tag name: tag index in param
tags_dict = dict()
for index, dict in enumerate(param_value):
    tags_dict[dict['name']] = index
    
print(tags_dict)


rospy.Subscriber('/tf', tfMessage, look_for_tag)
rospy.spin()

# save params to file 
with open(param_dir, 'r') as f:
    params = yaml.safe_load(f)
    params['standalone_tags'] = param_value
with open(param_dir, 'w') as f:
    yaml.safe_dump(params, f)
