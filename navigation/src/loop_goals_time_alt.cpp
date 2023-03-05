#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

#include <unistd.h> // for linux

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

#include <filesystem>
#include <ros/package.h>
#include <fstream>
#include <iomanip>

#include <ros/master.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <csignal>

#include <chrono>
#include <ctime>
#include <sstream>

#include <move_base_msgs/RecoveryStatus.h>

using namespace std;
namespace fs = std::filesystem;

struct MarkerParam
{
    string nspace;
    int type;
    double scale_x;
    double scale_y;
    double scale_z;
};

class Logger
{
public:
    Logger(const std::string &filename)
    {
        const char *home = getenv("HOME");
        if (home == NULL)
        {
            std::cerr << "HOME environment variable is not set" << std::endl;
            return;
        }
        std::string logger_file = std::string(home) + std::string(filename);
        ofs_.open(logger_file.c_str(), std::ios_base::out | std::ios_base::app);
    }

    ~Logger()
    {
        ofs_.close();
    }

    void log(const std::string &message)
    {
        std::time_t now = std::time(nullptr);
        std::tm *t = std::localtime(&now);
        ofs_ << std::put_time(t, "%Y-%m-%d %H:%M:%S") << ": " << message << std::endl;
        std::cout << message << std::endl;
    }

private:
    std::ofstream ofs_;
};

std::string logger_name = "/logger_goal.txt";
Logger logger(logger_name);

// Action specification for move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef boost::shared_ptr<geometry_msgs::PointStamped const> ptr_PointStamped;

ros::Publisher marker_pub;

FILE *ssrProcess = popen("simplescreenrecorder --start-recording --start-hidden >nul 2>nul", "w");
bool recording_started = false;

// Callback function for handling incoming messages
void recoveryStatusCallback(const move_base_msgs::RecoveryStatus::ConstPtr &msg)
{
    // Check if the message has any statuses
    // if (msg->status_list.size() > 0) {
    // Loop through the statuses and log the message for each one
    // for (const auto& status : msg->status_list) {
    logger.log("**************");
    logger.log("*Recovery status received: ");
    logger.log("*recovery was trigggered at: (" + std::to_string(msg->pose_stamped.pose.position.x) + ", " + std::to_string(msg->pose_stamped.pose.position.y) + ")");
    logger.log("*current_recovery_number: " + std::to_string(msg->current_recovery_number));
    logger.log("*total_number_of_recoveries: " + std::to_string(msg->total_number_of_recoveries));
    logger.log("*recovery_behavior_name: " + msg->recovery_behavior_name);
    logger.log("**************");
    // }
    // }
}

visualization_msgs::Marker get_marker(MarkerParam marker_param)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();

    // 设置 Marker 的 namespace 和 id。
    // 相同的 namespace 和 id 相同时新 marker 会覆盖旧值
    marker.ns = marker_param.nspace;
    marker.id = 0;

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

void set_pose(geometry_msgs::Pose &target, geometry_msgs::Point p, geometry_msgs::Quaternion q)
{
    target.position = p;
    target.orientation = q;
}

std::list<ptr_PointStamped> record_points()
{
    if (!ssrProcess)
    {
        printf("Failed to start SimpleScreenRecorder process.\n");
    }
    fprintf(ssrProcess, "record-start\n");
    fflush(ssrProcess);
    recording_started = true;

    MarkerParam points_param = {
        "points",
        visualization_msgs::Marker::POINTS,
        0.5, 0.5, 0.0};
    visualization_msgs::Marker points_marker = get_marker(points_param);

    std::list<ptr_PointStamped> points_list;

    logger.log("Received points: ");

    while (true)
    {
        // 监听 /clicked_point topic，获取点坐标，超时时间 5s
        // 超时返回值为空
        auto waypoint = ros::topic::waitForMessage<geometry_msgs::PointStamped>("/clicked_point", ros::Duration(5.0));

        if (waypoint == NULL)
        {
            cout << "\nThe point selection process is over." << endl;
            break;
        }

        ROS_INFO("input point is (%f, %f, %f)", waypoint->point.x, waypoint->point.y, waypoint->point.z);

        std::ostringstream messageStream;
        messageStream << "(" << waypoint->point.x << ", " << waypoint->point.y << ", " << waypoint->point.z << ")";
        std::string message = messageStream.str();

        logger.log(message);

        // 将标记点添加到列表中
        points_list.push_back(waypoint);

        // 生成并发布 Marker 在 rviz 中显示
        geometry_msgs::Point p = waypoint->point;

        points_marker.points.push_back(p);

        while (marker_pub.getNumSubscribers() < 1)
        {
            if (!ros::ok())
            {
                break;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        marker_pub.publish(points_marker);
    }

    logger.log("==============");

    return points_list;
}

move_base_msgs::MoveBaseGoal point_to_goal(
    std::list<ptr_PointStamped> &points_list,
    std::list<ptr_PointStamped>::iterator &it,
    std::list<ptr_PointStamped>::iterator &prev_iter,
    int &laps)
{
    geometry_msgs::Point prev;

    auto &point = (*it)->point;
    if (it != points_list.begin())
        prev = (*prev_iter)->point;

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

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

void log_amclpose()
{
    auto amcl_pose = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", ros::Duration(5.0));

    std::ostringstream messageStream;
    messageStream << "Current amcl_pose: (" << amcl_pose->pose.pose.position.x << ", " << amcl_pose->pose.pose.position.y << "), " << tf::getYaw(amcl_pose->pose.pose.orientation) << " degrees";
    std::string message = messageStream.str();

    logger.log(message);
}

void start_navigation(MoveBaseClient &ac, move_base_msgs::MoveBaseGoal goal, int max_retry)
{
    std::ostringstream messageStream;
    messageStream << "Current Goal: (" << goal.target_pose.pose.position.x << ", " << goal.target_pose.pose.position.y << "), " << tf::getYaw(goal.target_pose.pose.orientation) << " degrees";
    std::string message = messageStream.str();

logger.log("==============");
    logger.log(message);

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<move_base_msgs::RecoveryStatus>("/move_base/recovery_status", 4, recoveryStatusCallback);

    // 发送目标使小车开始移动，超时时间 180s
    ac.sendGoalAndWait(goal, ros::Duration(180.0, 0), ros::Duration(180.0, 0));

    while (1)
    {

        ros::spinOnce();
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("The robot has arrived at the goal location");
            logger.log("The robot has arrived at the goal location.");
            log_amclpose();
            logger.log("--------------");
            for (int i = 0; i < 5; i++)
            {
                sleep(1);
                ROS_INFO("waiting");
            }
            ROS_INFO("the wait is over");
            sub.shutdown();
            break;
        }
        else
        {
            switch (ac.getState().state_)
            {
            case actionlib::SimpleClientGoalState::PREEMPTED:
                logger.log("PREEMPTED");
                logger.log("The goal received a cancel request after it started executing and has since completed its execution");
                logger.log("--------------");
                break;
            case actionlib::SimpleClientGoalState::ABORTED:
                // max_retry = 0;
                logger.log("ABORTED");
                logger.log("The goal was aborted during execution by the action server due to some failure");
                logger.log("--------------");
                break;
            case actionlib::SimpleClientGoalState::RECALLED:
                logger.log("RECALLED");
                logger.log("The goal received a cancel request before it started executing and was successfully cancelled");
                logger.log("--------------");
                break;
            }
            log_amclpose();
            sub.shutdown();
            if (max_retry > 0)
            {
                logger.log("Retrying...");
                logger.log("Remaining retry count: " + std::to_string(max_retry));
                logger.log("--------------");
                start_navigation(ac, goal, --max_retry);
            }
            else
            {
                logger.log("The current goal may be unreachable, skipping the current goal");
                logger.log("--------------");
            }
                            
                break;
        }
    }
}



void resultCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    logger.log("result callback triggered");
}

// void stop_recording(FILE *ssrProcess)
void stop_recording()
{
    fprintf(ssrProcess, "record-save\n");
    fflush(ssrProcess);
    fprintf(ssrProcess, "quit\n");
    fflush(ssrProcess);

    pclose(ssrProcess);

    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    auto time_t = std::chrono::system_clock::to_time_t(now);

    // Convert to string
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d-%H-%M-%S.") << std::setfill('0') << std::setw(3) << ms.count();
    std::string dir_name = ss.str();

    const char *home = getenv("HOME");
    if (home == NULL)
    {
        std::cerr << "HOME environment variable is not set" << std::endl;
        return;
    }
    std::cout << "Home directory is: " << home << std::endl;

    fs::path logger_file = std::string(home) + logger_name;

    std::string package_path = ros::package::getPath("navigation");

    std::cout << "navigation path: " << package_path << std::endl;

    fs::path destDir = package_path + "/logs/" + dir_name;
    if (!fs::exists(destDir))
    {
        fs::create_directory(destDir);
    }

    try
    {
        // move the file to the destination directory
        // fs::rename(newestFile, destDir / newestFile.filename());
        fs::rename(logger_file, destDir / logger_file.filename());

        std::cout << "File moved successfully." << std::endl;
    }
    catch (const std::filesystem::filesystem_error &e)
    {
        std::cerr << "Error moving file: " << e.what() << std::endl;
    }

    if (recording_started)
    {
        std::string path = std::string(home) + "/视频";

        sleep(3);

        fs::directory_iterator dirIter(path);
        fs::path newestFile;
        std::filesystem::file_time_type newestTime = fs::last_write_time((*dirIter).path());
        for (auto &file : dirIter)
        {
            if (fs::is_regular_file(file.path()))
            {
                std::cout << "Current file: " << file.path() << std::endl;
                std::filesystem::file_time_type modifiedTime = fs::last_write_time(file.path());
                if (modifiedTime > newestTime)
                {
                    newestTime = modifiedTime;
                    newestFile = file.path();
                }
            }
        }
        std::cout << "Newest file: " << newestFile << std::endl;
        try
        {
            // move the file to the destination directory
            fs::rename(newestFile, destDir / newestFile.filename());
            // fs::rename(logger_file, destDir / logger_file.filename());

            std::cout << "File moved successfully." << std::endl;
        }
        catch (const std::filesystem::filesystem_error &e)
        {
            std::cerr << "Error moving file: " << e.what() << std::endl;
        }
    }

    ros::shutdown(); // Close the node
}

// Signal handler function
void signalHandler(int signum)
{
    std::cout << "Received signal " << signum << std::endl;
    // Perform any cleanup or shutdown actions here
    stop_recording();
    exit(signum);
}

int main(int argc, char **argv)
{
    logger.log("Hello, world!");

    signal(SIGINT, signalHandler);

    // 圈数
    int laps = 1;

    // Connect to ROS
    ros::init(argc, argv, "simple_navigation_goals");
    // tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);
    // Wait for the action server to come up so that we can begin processing goals.
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // Get the list of running ROS nodes
    std::vector<std::string> nodes;
    if (ros::master::getNodes(nodes))
    {
        for (const auto &node : nodes)
        {
            logger.log("Found node: " + std::string(node));
        }
    }
    else
    {
        ROS_ERROR("Failed to get list of running nodes.");
    }

    logger.log("==============");

    // Get the private namespace for move_base
ros::NodeHandle nh_priv("~move_base");

// Get all the parameters for move_base
XmlRpc::XmlRpcValue params;
if (nh_priv.getParam("", params))
{
  ROS_INFO("Successfully retrieved move_base parameters");
  ROS_INFO_STREAM("Parameters: " << params);
}
else
{
  ROS_WARN("Failed to retrieve move_base parameters");
}

    // 在 visualization_marker 上广播
    ros::NodeHandle n;
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Subscribe to the move_base/recovery_status topic
    // ros::Subscriber sub = n.subscribe<move_base_msgs::RecoveryStatus>("/move_base/recovery_status", 1, recoveryStatusCallback);

    // ros::Subscriber sub_ = n.subscribe("/amcl_pose", 1, resultCallback);

    cout << "\nPlease choose one point on map to start navigation." << endl;

    std::list<ptr_PointStamped> points_list = record_points();

    // while (ros::ok())
    // {
    //     ros::spinOnce();

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

        // ros::Duration(0.1).sleep();
    // }

    cout << "\n should stop recording" << endl;

    // stop_recording(ssrProcess);
    stop_recording();

    // ros::spin();

    return 0;
}
