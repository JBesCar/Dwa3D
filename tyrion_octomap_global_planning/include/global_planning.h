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
#include <ompl/base/DiscreteMotionValidator.h>
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
#include <octomap/octomap_types.h>
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

/*
    Class used to check that the motion between states are valid according to Octomap
*/
class octomapMotionValidator : public ompl::base::DiscreteMotionValidator
{
    public:
        octomapMotionValidator(const ompl::base::SpaceInformationPtr& space_info,
                                octomap::OcTree* octomap_, double safe_dist_ = -1) : ompl::base::DiscreteMotionValidator(space_info)
        {
            this->octomap = octomap_;
            this->safe_dist = safe_dist_;
        }
        virtual bool checkMotion(const ompl::base::State* s1,
                                const ompl::base::State* s2) const
        {   
            // Convert states to numeric info
            double x1 = s1->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
            double y1 = s1->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
            double z1 = s1->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];

            double x2 = s2->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
            double y2 = s2->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
            double z2 = s2->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];

            // Compute the motion direction
            double dx = x2 - x1;
            double dy = y2  - y1;
            double dz = z2 - z1;

            // Ray casting
            octomap::point3d origin = octomap::point3d(x1, y1, z1);
            octomap::point3d ray = octomap::point3d(dx, dy, dz);
            octomap::point3d end;
            double length = std::sqrt((dx * dx) + (dy * dy) + (dz * dz));
            bool occupied = octomap->castRay(origin, ray, end, true, length);
            //If is size aware, cast multiple rays to take size into account
            if(safe_dist > 0 && !occupied){
                double theta = atan2(dy, dx);
                for(double i = -1; i < 1.1; i = i + 0.5){
                    for(double j = -1; j < 1.1; j = j + 0.5){
                        octomap::point3d origin = octomap::point3d(x1 - i * safe_dist * sin(theta), 
                                                                    y1 + i * safe_dist * cos(theta), 
                                                                    z1 + j * safe_dist);
                        occupied = occupied || octomap->castRay(origin, ray, end, true, length);
                    }
                }
            }

            return !occupied;
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
        double safe_dist; 
};


/*
    Class used to compute the height alignment term in the global planning
*/
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


/*
    Class that is in charge of performing the global planning with OMPL
*/
class GlobalPlanner{
    private:
        // TO-DO: Planner bounds
        double XMIN, XMAX, YMIN, YMAX, ZMIN, ZMAX; //-20.0//85.0//-24.5
        double safety_distance;
        double k_length, k_heading;
        // Topics Names
        std::string odom_topic;
        std::string goal_topic;
        std::string octomap_topic;
        std::string markers_path_topic;
        std::string waypoints_topic;
        std::string default_goal_topic = "/vrpn_client_node/goal_optitrack/pose";
        std::string default_odom_topic = "/optitrack/pose";
        std::string default_octomap_topic = "/octomap_binary";
        std::string default_markers_path_topic = "/path";
        std::string default_waypoints_topic = "/waypoint_list";



        // Current Position and Goal Position
        bool odom_received, goal_recieved, enable_replan;
        geometry_msgs::Pose goal;
        geometry_msgs::PoseStamped current_pose;

        // Visual info
        visualization_msgs::Marker marker_msg, marker_lines_msg;

        // Publishers and Subscribers
        ros::Publisher path_pub, waypoints_pub;
        ros::Subscriber pose_sub, goal_sub, octomap_sub;

        // Map
        ros::ServiceClient octomap_server;
        octomap_msgs::Octomap last_octomap_msg;
        octomap::OcTree* octomap;
        bool octomap_recieved;

        // Callbacks
        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void octomapCallback(octomap_msgs::Octomap msg);

        // Process Octomap msg
        octomap::OcTree *getOctomap(void);

        // Plan with OMPL
        std::vector<geometry_msgs::Point> plan(geometry_msgs::Pose start_pose, geometry_msgs::Pose end_pose);
        double max_planning_time, max_segment_length;

        // Init Visual Info
        void initMarkerMsgs(void);

        // Unused
        double goalDistance(geometry_msgs::Pose pose, geometry_msgs::Point goal);        
        
    public:
        // Constructor
        GlobalPlanner(ros::NodeHandle& nh);

        // "Main" function
        void run(void);

        // State Validity Checker for OMPL planning 
        bool isStateValid(const ompl::base::State *state);

};
