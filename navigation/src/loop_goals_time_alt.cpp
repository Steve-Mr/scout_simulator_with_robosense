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

#include <geometry_msgs/Pose.h>

using namespace std;

struct MarkerParam
{
    string nspace;
    int type;
    double scale_x;
    double scale_y;
    double scale_z;
};

// Action specification for move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef boost::shared_ptr<geometry_msgs::PointStamped const> ptr_PointStamped;

ros::Publisher marker_pub;

visualization_msgs::Marker get_marker(MarkerParam marker_param)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    
    // 设置 Marker 的 namespace 和 id。 
    // 相同的 namespace 和 id 相同时新 marker 会覆盖旧值
    marker.ns = marker_param.nspace;
    marker.id =0;

    // 动作：支持添加 ADD/删除 DELETE/删除全部 DELETEALL
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    
    marker.type = marker_param.type;
    
    // Marker 的比例，这里 1.0 为 1m
    marker.scale.x = marker_param.scale_x;
    marker.scale.y = marker_param.scale_y;
    marker.scale.z = marker_param.scale_z;
    
    // 设置 Marker 颜色（此处为绿色
    marker.color.g = 1.0f;
    marker.color.a = 1.0;

    return marker;
}

void set_pose(geometry_msgs::Pose& target, geometry_msgs::Point p ,geometry_msgs::Quaternion q)
{
    target.position = p;
    target.orientation = q;
}

std::list<ptr_PointStamped> record_points()
{
    MarkerParam points_param = {
        "points",
        visualization_msgs::Marker::POINTS,
        0.5, 0.5, 0.0};
    visualization_msgs::Marker points_marker = get_marker(points_param);

    std::list<ptr_PointStamped> points_list;

    while(true){
    	// 监听 /clicked_point topic，获取点坐标，超时时间 5s
    	// 超时返回值为空
    	auto waypoint = ros::topic::waitForMessage<geometry_msgs::PointStamped>("/clicked_point", ros::Duration(5.0));   
    		
    	if(waypoint == NULL){
			cout<<"\nThe point selection process is over."<<endl;
			break;
    	}
    		
   		ROS_INFO("input point is (%f, %f, %f)", waypoint->point.x, waypoint->point.y, waypoint->point.z);

		// 将标记点添加到列表中
		points_list.push_back(waypoint);
    	
    	// 生成并发布 Marker 在 rviz 中显示
    	geometry_msgs::Point p = waypoint->point;

		points_marker.points.push_back(p);
	
		while (marker_pub.getNumSubscribers() < 1){
      		if (!ros::ok()){
        		break;
      		}
      		ROS_WARN_ONCE("Please create a subscriber to the marker");
    		sleep(1);
		}
		marker_pub.publish(points_marker);    	
    }

    return points_list;
}

move_base_msgs::MoveBaseGoal point_to_goal(
    std::list<ptr_PointStamped>& points_list,
    std::list<ptr_PointStamped>::iterator& it,
    std::list<ptr_PointStamped>::iterator& prev_iter,
    int& laps)
{
    geometry_msgs::Point prev;

    auto &point = (*it)->point;
    if (it != points_list.begin())
        prev = (*prev_iter)->point;

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    ROS_INFO("goal is %f", point.x);
    ROS_INFO("goal is %f", point.y);

    double theta = atan2(point.y - prev.y, point.x - prev.x);

    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(theta);
    set_pose(goal.target_pose.pose, point, q);

    MarkerParam arrow_param = {
        "arrow",
        visualization_msgs::Marker::ARROW,
        1.0, 0.2, 0.2};

    visualization_msgs::Marker arrow_marker = get_marker(arrow_param);
    set_pose(arrow_marker.pose, point, q);
    marker_pub.publish(arrow_marker);

    prev_iter = it;
    it++;
    if (it == points_list.end() && laps != 1)
    {
        laps--;
        it = ++points_list.begin();
        prev_iter = points_list.begin();
    }

    return goal;
}

void start_navigation(MoveBaseClient &ac, move_base_msgs::MoveBaseGoal goal, int max_retry)
{

    // 发送目标使小车开始移动，超时时间 180s
    ac.sendGoalAndWait(goal, ros::Duration(180.0, 0), ros::Duration(180.0, 0));

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The robot has arrived at the goal location");
        for (int i = 0; i < 5; i++)
        {
            sleep(1);
            ROS_INFO("waiting");
        }
        ROS_INFO("the wait is over");
    }
    else
    {
        ROS_INFO("The robot may have failed to reach the goal location");
        if (max_retry > 0)
        {
            start_navigation(ac, goal, --max_retry);
        }
    }
}

int main(int argc, char** argv){

	// 圈数
	int laps = 3;

    // Connect to ROS
    ros::init(argc, argv, "simple_navigation_goals");
  
    // tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);
  
    // Wait for the action server to come up so that we can begin processing goals.
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
        
    // 在 visualization_marker 上广播
    ros::NodeHandle n;
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    std::list<ptr_PointStamped> points_list = record_points();
    
    cout << "\nPlease choose one point on map to start navigation."<<endl;

    if (points_list.size() != 0)
    {
        // 超过一个标记点：设置终点为第一个标记点
        if (points_list.size() != 1)
            points_list.push_back(points_list.front());

        auto prev_iter = points_list.begin();
        auto it = points_list.begin();
        while (it != points_list.end())
        {

            ROS_INFO("%d laps left", laps);

            move_base_msgs::MoveBaseGoal goal = point_to_goal(points_list, it, prev_iter, laps);

            cout << "\nGoing to next point." << endl;

            start_navigation(ac, goal, 3);
        }
    }

  return 0;
}
