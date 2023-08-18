#!/usr/bin/env python3

import rospy
import yaml
from tf.msg import tfMessage
import tf2_ros

import suppress_warnings

rospy.init_node('tag_logger')
suppress_warnings.suppress_TF_REPEATED_DATA()

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

def look_for_tag(msg):
    for transform in msg.transforms:
        frame_name = transform.child_frame_id
        if frame_name in tags_dict.keys():
            print(f"Detected new TF frame: {frame_name}")

            # tf2 ver
            try:
                trans = tfBuffer.lookup_transform('map', frame_name, rospy.Time())
                print(trans)
                param_value[tags_dict[frame_name]]['x'] = trans.transform.translation.x
                param_value[tags_dict[frame_name]]['y'] = trans.transform.translation.y
                param_value[tags_dict[frame_name]]['z'] = trans.transform.translation.z
                param_value[tags_dict[frame_name]]['qx'] = trans.transform.rotation.x
                param_value[tags_dict[frame_name]]['qy'] = trans.transform.rotation.y
                param_value[tags_dict[frame_name]]['qz'] = trans.transform.rotation.z
                param_value[tags_dict[frame_name]]['qw'] = trans.transform.rotation.w
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
