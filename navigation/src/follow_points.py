#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def send_goal_and_wait(client, goal):
    """Send a goal and wait for the result."""
    client.send_goal(goal)
    client.wait_for_result()

def main():
    # 初始化 ros 节点
    rospy.init_node('send_goals')

    # 建立 SimpleActionClient 与 move_base action server 交互 
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # 从文件中按行读取点
    filename = rospy.get_param("~filename")
    with open(filename) as file:
        for line in file:
            fields = line.split()
            if len(fields) != 7:
                rospy.logwarn("Invalid goal format: {}".format(line))
                continue
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.pose.position.x = float(fields[0])
            goal.target_pose.pose.position.y = float(fields[1])
            goal.target_pose.pose.position.z = float(fields[2])
            goal.target_pose.pose.orientation.x = float(fields[3])
            goal.target_pose.pose.orientation.y = float(fields[4])
            goal.target_pose.pose.orientation.z = float(fields[5])
            goal.target_pose.pose.orientation.w = float(fields[6])
            
            print('current goal: ({}, {})'.format(fields[0], fields[1]))

            send_goal_and_wait(client, goal)
            
            print('goal reached.')

            rospy.sleep(2)

if __name__ == '__main__':
    main()
