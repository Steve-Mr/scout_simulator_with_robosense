#!/usr/bin/env python3

import rospy
import os
from geometry_msgs.msg import PoseWithCovarianceStamped

class Pose:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        
class Orientation:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.w = 0
        

class RobotPosition:
    def __init__(self):
        pose = Pose()
        orientation = Orientation()
        self.pose = pose
        self.orientation = orientation

    def callback(self, data):
        self.pose.x = data.pose.pose.position.x
        self.pose.y = data.pose.pose.position.y
        self.pose.z = data.pose.pose.position.z
        self.orientation.x = data.pose.pose.orientation.x
        self.orientation.y = data.pose.pose.orientation.y
        self.orientation.z = data.pose.pose.orientation.z
        self.orientation.w = data.pose.pose.orientation.w



def write_pose_to_file(filename: str, pose_msg: RobotPosition):
    # 获取文件路径
    dir_path = os.path.dirname(os.path.abspath(__file__))
    results_dir = os.path.join(dir_path, 'results')

    # 如果 results 文件夹不存在则创建
    if not os.path.exists(results_dir):
        os.makedirs(results_dir)

    # 构造文件路径
    file_path = os.path.join(results_dir, filename)

    # 将 x, y, z, w 写入文件
    with open(file_path, 'a') as f:
        f.write('{:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f}\n'.format(
                pose_msg.pose.x, pose_msg.pose.y, pose_msg.pose.z, pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w))


if __name__ == '__main__':
    
    file_name = input('Please enter file name to store points list: ')
    
    rospy.init_node('robot_position')

    position = RobotPosition()
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, position.callback)

    while not rospy.is_shutdown():
        command = input('Please enter command (0 to exit): ')
        if command == '0':
            break
        elif command == '1':
            write_pose_to_file(file_name, position)
            print('Robot position: ({:.2f}, {:.2f}, {:.2f}), ({:.2f}, {:.2f}, {:.2f}, {:.2f})'.format(
                position.pose.x, position.pose.y, position.pose.z, position.orientation.x, position.orientation.y, position.orientation.z, position.orientation.w))
        else:
            print('Unknown command.')
    
    rospy.signal_shutdown('Exit.')
