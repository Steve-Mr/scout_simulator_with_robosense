#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped

def send_goal_and_wait(client, goal):
    """Send a goal and wait for the result."""
    client.send_goal(goal)
    client.wait_for_result()
    
def amcl_pose_callback(msg):
    # 处理接收到的 amcl_pose 信息
    pose = msg.pose.pose
    position = pose.position
    orientation = pose.orientation

    # 输出位置和姿态信息
    print("AMCL Pose - Position: [x: %f, y: %f, z: %f], Orientation: [x: %f, y: %f, z: %f, w: %f]",
                  position.x, position.y, position.z,
                  orientation.x, orientation.y, orientation.z, orientation.w)


def main():
    # 初始化 ros 节点
    rospy.init_node('send_goals')

    # 建立 SimpleActionClient 与 move_base action server 交互 
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # 打开文件并读取所有行
    filename = rospy.get_param("~filename")
    with open(filename, "r") as file:
        lines = file.readlines()

    # 将每行按照在文件中的存储顺序进行编号后显示在屏幕上
    for i, line in enumerate(lines):
        print(f"{i+1}. {line.strip()}")

    # 获取用户输入并根据输入的编号将对应的行重新打包起来
    input_str = input("请输入需要的编号：")
    selected_indices = [int(index.strip())-1 for index in input_str.replace('，', ',').split(",")]

    selected_lines = [lines[i].strip() for i in selected_indices]
    lines_str = "\n".join(selected_lines)
    print(f"您选择的点有：\n{lines_str}")

    # 根据重新组合后的行进行导航
    for goalindex, line in enumerate(selected_lines):
        print("\nnew line start: ")
        print(line)
        print("current position: ")
        msg = rospy.wait_for_message('amcl_pose', PoseWithCovarianceStamped)  
        pose = msg.pose.pose
        position = pose.position
        orientation = pose.orientation

        # 输出位置和姿态信息
        print("AMCL Pose - Position: [x: %f, y: %f, z: %f], Orientation: [x: %f, y: %f, z: %f, w: %f]",
                  position.x, position.y, position.z,
                  orientation.x, orientation.y, orientation.z, orientation.w)      
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
        status = client.get_state()
        if status == actionlib.GoalStatus.SUCCEEDED:
            print('point index ' + str(selected_indices[goalindex]+1) + ' goal reached')
        else:
            print('point index ' + str(selected_indices[goalindex]+1) + ' goal not reatched')

        rospy.sleep(2)

if __name__ == '__main__':
    main()
