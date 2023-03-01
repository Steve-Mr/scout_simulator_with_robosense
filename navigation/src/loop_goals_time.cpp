#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

#include <unistd.h>               // for linux 

#include <boost/shared_ptr.hpp>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h> 

#include <visualization_msgs/Marker.h>

#include <list>

#include <cmath>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>

using namespace std;

// Action specification for move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){

    // Connect to ROS
    ros::init(argc, argv, "simple_navigation_goals");
  
    // tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);
  
    // Wait for the action server to come up so that we can begin processing goals.
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    move_base_msgs::MoveBaseGoal goal;
    
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    
    boost::shared_ptr<geometry_msgs::PointStamped const> waypoint;
	boost::shared_ptr<geometry_msgs::PointStamped const> prev_waypoint;
    std::list<boost::shared_ptr<geometry_msgs::PointStamped const>> points_list;
    
    // 在 visualization_marker 上广播
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    
    visualization_msgs::Marker points_marker;
    points_marker.header.frame_id = "map";
    points_marker.header.stamp = ros::Time();
    
    // 设置 Marker 的 namespace 和 id。 
    // 相同的 namespace 和 id 相同时新 marker 会覆盖旧值
    points_marker.ns = "points";
    points_marker.id =0;

    // 动作：支持添加 ADD/删除 DELETE/删除全部 DELETEALL
    points_marker.action = visualization_msgs::Marker::ADD;
    points_marker.pose.orientation.w = 1.0;
    
    points_marker.type = visualization_msgs::Marker::POINTS;
    
    // Marker 的比例，这里 1.0 为 1m
    points_marker.scale.x = 1.0;
    points_marker.scale.y = 1.0;
    points_marker.scale.z = 1.0;
    
    // 设置 Marker 颜色（此处为绿色
    points_marker.color.g = 1.0f;
    points_marker.color.a = 1.0;
    
    cout << "\nPlease choose one point on map to start navigation."<<endl;
    
    while(true){
    	// 监听 /clicked_point topic，获取点坐标，超时时间 5s
    	// 超时返回值为空
    	waypoint = ros::topic::waitForMessage<geometry_msgs::PointStamped>("/clicked_point", ros::Duration(5.0));   
    		
    	if(waypoint == NULL){
			cout<<"\nThe point selection process is over."<<endl;
			break;
    	}
    		
   		ROS_INFO("point.x is %f", waypoint->point.x);
   		ROS_INFO("point.y is %f", waypoint->point.y);

		// 将标记点添加到列表中
		points_list.push_back(waypoint);

		// double theta = 0.0;
		// if (points_list.size() != 1)
		// 	theta = atan2(waypoint->point.y - prev_waypoint->point.y, waypoint->point.x - prev_waypoint->point.x);
		// else
		// {
		// 	theta = atan2(waypoint->point.y - 0.0, waypoint->point.x - 0.0);
		// }
		// prev_waypoint = waypoint;
		// geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(theta);
		// points_marker.pose.orientation = q;
    	
    	// 生成并发布 Marker 在 rviz 中显示
    	geometry_msgs::Point p;
		p.x = waypoint->point.x;
		p.y = waypoint->point.y;
		p.z = waypoint->point.z;

		points_marker.points.push_back(p);
	
		while (marker_pub.getNumSubscribers() < 1){
      		if (!ros::ok()){
        		return 0;
      		}
      		ROS_WARN_ONCE("Please create a subscriber to the marker");
    		sleep(1);
		}
		marker_pub.publish(points_marker);    	
    }
    
    if(points_list.size() != 0){
    	// 超过一个标记点：设置终点为第一个标记点
    	// if(points_list.size() != 1){
    	// 	points_list.push_back(points_list.front());
    	// }
		auto prev_iter = points_list.begin();
    	for(auto it = points_list.begin(); it != points_list.end(); ++it){
			auto const& point = *it;
			auto const& prev = *prev_iter;
			
    		ROS_INFO("goal is %f", point->point.x);
    		ROS_INFO("goal is %f", point->point.y);
    		
			double theta = 0;

			if (it != points_list.begin()){
				theta = atan2(point->point.y - prev->point.y, point->point.x - prev->point.x);
				ROS_INFO("prev.x is %f", prev->point.x);
				ROS_INFO("prev.y is %f", prev->point.y);}
			else
			{
				theta = atan2(point->point.y - 0.0, point->point.x - 0.0);
			}
			
			prev_iter = it;
			geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(theta);

			goal.target_pose.pose.position.x = point->point.x;
    		goal.target_pose.pose.position.y = point->point.y;
			goal.target_pose.pose.orientation = q;

			visualization_msgs::Marker arrow_marker;
			arrow_marker.header.frame_id = "map";
			arrow_marker.header.stamp = ros::Time();

			// 设置 Marker 的 namespace 和 id。
			// 相同的 namespace 和 id 相同时新 marker 会覆盖旧值
			arrow_marker.ns = "points";
			arrow_marker.id = 0;

			// 动作：支持添加 ADD/删除 DELETE/删除全部 DELETEALL
			arrow_marker.action = visualization_msgs::Marker::ADD;
			arrow_marker.pose.orientation.w = 1.0;

			arrow_marker.type = visualization_msgs::Marker::ARROW;

			// Marker 的比例，这里 1.0 为 1m
			arrow_marker.scale.x = 1.0;
			arrow_marker.scale.y = 0.2;
			arrow_marker.scale.z = 0.0;

			// 设置 Marker 颜色（此处为绿色
			arrow_marker.color.g = 1.0f;
			arrow_marker.color.a = 1.0;

			arrow_marker.pose.position.x = point->point.x;
			arrow_marker.pose.position.y = point->point.y;
			arrow_marker.pose.position.z = 0;
			arrow_marker.pose.orientation = q;

			marker_pub.publish(arrow_marker);

			cout << "\nGoing to next point." << endl;
    
    		// 发送目标使小车开始移动，超时时间 180s
    		ac.sendGoalAndWait(goal, ros::Duration(180.0,0), ros::Duration(180.0,0));
 
    		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      			ROS_INFO("The robot has arrived at the goal location");
      			for(int i = 0; i < 9; i++){
      				sleep(1);
      				ROS_INFO("waiting");
      			}
      			//sleep(10);
      			ROS_INFO("the wait is over");
      			}
    		else
      			ROS_INFO("The robot may have failed to reach the goal location");
    	}
    }
    
  return 0;
}
