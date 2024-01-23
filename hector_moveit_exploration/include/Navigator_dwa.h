#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/conversions.h>


#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <hector_moveit_actions/ExecuteDroneTrajectoryAction.h>

#include <octomap/OcTree.h>


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
typedef hector_moveit_actions::ExecuteDroneTrajectoryResult Result;

#include <iostream>
#include <chrono>

using namespace std;
using  ns = chrono::nanoseconds;
using get_time = chrono::steady_clock;

#include <omp.h>
#define PATCH_LIMIT 1
class Navigator_dwa{
    private:
        // Cueva  || Tunel || Orchyard
        double XMIN, XMAX, YMIN, YMAX, ZMIN, ZMAX; //-20.0//85.0//-24.5
        std::string globalPanner;
        std::string defaultPlanner = "RRTConnectkConfigDefault";
        std::string odom_topic;
        std::string goal_topic;
        std::string default_goal_topic = "/vrpn_client_node/goal_optitrack/pose";
        std::string default_odom_topic = "/vrpn_client_node/base_link/pose";
        //#define XMAX 5.0//120.0//24.5
        //#define YMIN -25.0//-5.0//-16
        //#define YMAX 20.0//30.0//16
        //#define ZMIN 0.2
        //#define ZMAX 15.0
        std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
        actionlib::SimpleActionClient<hector_moveit_actions::ExecuteDroneTrajectoryAction> trajectory_client;
        std::unique_ptr<robot_state::RobotState> start_state;
        std::unique_ptr<planning_scene::PlanningScene> planning_scene;
        robot_model::RobotModelPtr kmodel;
        const double takeoff_altitude = 1.0;
        int GRID;
        bool odom_received,trajectory_received,goal_recieved;
        bool isPathValid;
        bool collision;

        geometry_msgs::Pose odometry_information, goal;
        std::vector<geometry_msgs::Pose> trajectory;

        mavros_msgs::State current_state;

        std::vector<geometry_msgs::Pose> invalid_poses;
        //std::vector<std::vector<int> > patches;
        std::queue<DistancedPoint> frontiers;
        std::vector<geometry_msgs::Pose> explored;

        ros::Subscriber base_sub,plan_sub,goal_sub;
        ros::ServiceClient planning_scene_service;

        moveit_msgs::RobotState plan_start_state;
        moveit_msgs::RobotTrajectory plan_trajectory;
    
        const std::string PLANNING_GROUP = "Quad_base";

        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

        void planCallback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg);

        void collisionCallback(const hector_moveit_actions::ExecuteDroneTrajectoryFeedbackConstPtr& feedback);

        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

        // double countFreeVolume(const octomap::OcTree *octree);
        double calc_MI(const octomap::OcTree *octree, const geometry_msgs::Point& point, const octomap::Pointcloud &hits, const double before);
        octomap::Pointcloud castSensorRays(const octomap::OcTree* curr_tree,const geometry_msgs::Pose& pose);
        //void findFrontier();

        bool go(geometry_msgs::Pose& target_);
        double goalDistance(geometry_msgs::Pose pose, geometry_msgs::Point goal);
        void enterRecoveryMode(int n_searchs_recovery, double search_dist);
        robot_state::RobotState searchSafePositionAround(int n_searchs_recovery, double search_dist);
    
    public:
        Navigator_dwa(ros::NodeHandle& nh);
        void run(void);
};
