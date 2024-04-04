#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>


#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <std_srvs/Empty.h>

//OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/State.h>
#include <ompl/base/Cost.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


//OctoMap
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_server/OctomapServer.h>



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


class octomapMotionValidator : public ompl::base::MotionValidator
{
    public:
        octomapMotionValidator(const ompl::base::SpaceInformationPtr& space_info,
                                octomap::OcTree* octomap_) : ompl::base::MotionValidator(space_info)
        {
            this->octomap = octomap_;
        }
        virtual bool checkMotion(const ompl::base::State* s1,
                                const ompl::base::State* s2) const
        {
            double x1 = s1->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
            double y1 = s1->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
            double z1 = s1->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];

            double x2 = s2->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
            double y2 = s2->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
            double z2 = s2->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];

            double dx = x2 - x1;
            double dy = y2  - y1;
            double dz = z2 - z1;

            octomap::point3d origin = octomap::point3d(x1, y1, z1);
            octomap::point3d ray = octomap::point3d(dx, dy, dz);
            octomap::point3d end;

            double length = std::sqrt((dx * dx) + (dy * dy) + (dz * dz));

            return !octomap->castRay(origin, ray, end, true, length);
        } 

        virtual bool checkMotion(const ompl::base::State* s1,
                                const ompl::base::State* s2,
                                std::pair<ompl::base::State*, double>& last_valid) const
        {   
            ROS_WARN("NOT IMPLEMENTED");
            return false;
        }
    private:
        octomap::OcTree* octomap;  
};

class HeadingObjective : public ompl::base::StateCostIntegralObjective
{
    public:
        HeadingObjective(const ompl::base::SpaceInformationPtr& si,
                        ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal_) : 
                        ompl::base::StateCostIntegralObjective(si, true),
                        goal(goal_)
        {
            
        }

        ompl::base::Cost stateCost(const ompl::base::State* s) const
        {   
            const auto queryPosition = s->as<ompl::base::RealVectorStateSpace::StateType>();
            double z_state = queryPosition->values[2];
            return ompl::base::Cost(std::fabs(goal[2] - z_state));
        }
    private:
        ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal;
};


class GlobalPlanner{
    private:
        double XMIN, XMAX, YMIN, YMAX, ZMIN, ZMAX; //-20.0//85.0//-24.5
        std::string odom_topic;
        std::string goal_topic;
        std::string default_goal_topic = "/vrpn_client_node/goal_optitrack/pose";
        std::string default_odom_topic = "/optitrack/pose";
        //#define XMAX 5.0//120.0//24.5
        //#define YMIN -25.0//-5.0//-16
        //#define YMAX 20.0//30.0//16
        //#define ZMIN 0.2
        //#define ZMAX 15.0
        const double takeoff_altitude = 1.0;
        int GRID;
        bool odom_received,trajectory_received,goal_recieved;
        bool isPathValid;
        bool collision;

        geometry_msgs::Pose goal;
        geometry_msgs::PoseStamped odometry_information;
        std::vector<geometry_msgs::Pose> trajectory;
        visualization_msgs::Marker marker_msg, marker_lines_msg;


        mavros_msgs::State current_state;

        std::vector<geometry_msgs::Pose> invalid_poses;

        ros::Publisher path_pub, waypoints_pub;
        ros::Subscriber base_sub,plan_sub,goal_sub,octomap_sub;
        ros::ServiceClient planning_scene_service;
        std::string planner_service;
        std::string publish_plath_service;

        // Map
        ros::ServiceClient octomap_server;
        octomap_msgs::Octomap last_octomap_msg;
        octomap::OcTree* octomap;
        bool octomap_recieved;

 

        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

        void planCallback(const  visualization_msgs::MarkerArray::ConstPtr &msg);

        //void collisionCallback(const &feedback);

        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void octomapCallback(octomap_msgs::Octomap msg);
        octomap::OcTree *getOctomap(void);

        bool go(geometry_msgs::Pose& target_);
        double goalDistance(geometry_msgs::Pose pose, geometry_msgs::Point goal);
        std::vector<geometry_msgs::Point> plan(geometry_msgs::Pose start_pose, geometry_msgs::Pose end_pose);
        
        //void enterRecoveryMode(int n_searchs_recovery, double search_dist);
        //robot_state::RobotState searchSafePositionAround(int n_searchs_recovery, double search_dist);
        void initMarkerMsgs(void);
    public:
        GlobalPlanner(ros::NodeHandle& nh);
        void run(void);
        bool isStateValid(const ompl::base::State *state);

};
