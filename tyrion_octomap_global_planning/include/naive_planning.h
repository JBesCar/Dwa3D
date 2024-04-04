#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>


#define _USE_MATH_DEFINES
#include <cmath>
#include <queue>

#define EPSILON 1e-4

typedef std::pair<double,geometry_msgs::Pose> DistancedPoint;

class Compare{
    public: 
        bool operator()(DistancedPoint& lhs, DistancedPoint & rhs)
        {
            return lhs.first < rhs.first;
        }
};
typedef std::priority_queue<DistancedPoint,std::vector<DistancedPoint>, Compare> DistancedPointPriorityQueue;

#include <iostream>
#include <chrono>

using namespace std;
using  ns = chrono::nanoseconds;
using get_time = chrono::steady_clock;

#include <omp.h>
#define PATCH_LIMIT 1
class NaivePlanner{
    private:
        std::string odom_topic;
        std::string goal_topic;
        std::string waypoints_topic;
        std::string default_goal_topic = "/vrpn_client_node/goal_optitrack/pose";
        std::string default_odom_topic = "/optitrack/pose";
        std::string default_waypoints_topic = "/waypoint_list";
        bool odom_received,trajectory_received,goal_recieved;

        geometry_msgs::PoseStamped odometry_information, goal;
        std::vector<geometry_msgs::Pose> trajectory;

        std::vector<geometry_msgs::Pose> invalid_poses;

        ros::Subscriber base_sub,goal_sub;
        ros::Publisher waypoints_pub;
        ros::ServiceClient planning_scene_service;
        std::string planner_service;
        std::string publish_plath_service;

 

        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

        //void collisionCallback(const &feedback);

        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

        bool go(geometry_msgs::Pose& target_);
        double goalDistance(geometry_msgs::Pose pose, geometry_msgs::Point goal);
        //void enterRecoveryMode(int n_searchs_recovery, double search_dist);
        //robot_state::RobotState searchSafePositionAround(int n_searchs_recovery, double search_dist);
    
    public:
        NaivePlanner(ros::NodeHandle& nh);
        void run(void);
};
