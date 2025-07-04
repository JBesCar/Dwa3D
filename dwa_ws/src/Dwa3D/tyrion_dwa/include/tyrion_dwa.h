#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32.h>

#include <cmath>
#define _USE_MATH_DEFINES

#define MAX_SPEED 1.5
#define EPSILON 1e-4

//MAVROS
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/SetMavFrame.h>


// DWA
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tyrion_dwa/DynamicWindowMsg.h>

//OctoMap
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_server/OctomapServer.h>

#include <array>

#define PI 3.14159265
#define COLS 8


class Dwa3d {
    private:
        ros::NodeHandle nh_, nh_private_;
        ros::Publisher vel_pub, DWA_visual_pub, comp_time_pub;
        ros::Subscriber pose_sub, state_sub, extended_state_sub, current_vel_sub, plan_sub, octomap_sub;
        ros::Publisher markers_debug_pub, predicted_pose_pub, discarded_poses_pub, vel_visual_pub;
        ros::ServiceClient set_cmd_vel_frame, arming_client, set_mode_client;

        geometry_msgs::Twist empty,cmd, current_vel_msg;
        geometry_msgs::TwistStamped vel_visual_msg;
        std::vector<geometry_msgs::Pose> trajectory;
        geometry_msgs::Pose current_pose;
       
        bool executing;

        //MAVROS
        mavros_msgs::SetMode mavros_set_mode;
        mavros_msgs::CommandBool arm_cmd;
        mavros_msgs::State current_state;
        mavros_msgs::ExtendedState current_extended_state;

        // DWA
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        double ALFA, Ky, Kz, BETA, GAMMA;
        double step, subgoal_step;
        int goal_i;
        bool lecturaObstaculos = true;
        bool current_vel_recieved = false;
        bool pose_recieved = false;
        bool octomap_recieved = false;
        bool treat_unknown_as_occupied = true;
        const double R_drone; //Drone Radius [m], Default = 0.5
        const double T; //Control Period [s], Default = 0.1 
        const double delta_t; //Prediction Horizon [s], Default = 1
        const double vx_step; //Search Space Discretization vx[m/s], Default = 0.05
        const double vz_step; //Search Space Discretization vz[m/s], Default = 0.05
        const double w_step; // //Search Space Discretization w[rad/s], Default = Pi/36 rad/s = 5 deg/s
        const double aLin; // Maximum Linear Acceleration [m/ss], Default = 1.0
        const double aAng; // Maximum Angular acceleration [rad/ss] Default = Pi/1.8 rad/ss = 100 deg/ss
        const int filas_tot;
        int iter_obs; //iter_obs*T = delta_t
        int iter_update;
        
        // Objective function and its terms
        std::vector<std::array<double, COLS>> comp_eval;
        std::vector<std::array<double, COLS>> comp_eval_norm; // For each velocity of the window
        std::vector<double> G;

        // Velocity limits 
        std::array<double,6> Vs;// = { 0, vx_max, -w_max, w_max, -vz_max, vz_max }; // Maximum Velocities Search Space, V
        double vx_min = 0.0;
        double vx_max = 0.3; //Default = 0.3
        double vz_max = 0.3; //Default = 0.3
        double w_max = PI/4; // Default = PI/4

        //Ray casting parameters
        double r_search;
        double psi_beam_max = PI/2; //Azimuthal Angular Raycasting limits [rad], Default = PI/2 rad = 180 deg
        double theta_beam_max = PI/2; //Elevation Angular Raycasting limits [rad], Default = PI/2 rad = 180 deg
        double delta_psi = 10 * PI/180; //Azimuthal Angular Resolution Raycasting [rad], Default = PI/18 rad = 10 deg
        double delta_theta = 10 * PI/180; //Elevation Angular Resolution Raycasting [rad], Default = PI/18 rad = 10 deg
        double lambda_psi = 0.5; // Weight , Default = 0.5
        double lambda_theta = 0.75; //, Default = 0.75

        // Map
        octomap_msgs::Octomap last_octomap_msg;
        octomap::OcTree* octomap;

        // Topics and frames
        std::string cmd_vel_control_topic, pose_topic, plan_topic,
                    current_vel_topic, cmd_frame;


    public:
        Dwa3d(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, 
            const double _R_drone, const double _T, const double _delta_t,
            const double _vx_step, const double _vz_step, const double _w_step, 
            const double _aLin, const double _aAng);
            
        void state_cb(const mavros_msgs::State::ConstPtr& msg);

        void extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg);
            
        void planCallback(const geometry_msgs::PoseArray::ConstPtr& path);

        void followPlan();

        void idle();

        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg);
        
        void currentVelCallback(const geometry_msgs::TwistStamped msg);


        /*
            Function that is in charge of retrieving the octomap from the server
            and performing the needed transformations.
        */
        void octomapCallback(octomap_msgs::Octomap msg);
        octomap::OcTree* getOctomap();

        double norm_rad(double rad);
        double rad2deg(double rad);
        
        
        /*
            Computes the distance to the nearest voxel of the Octotree given a pose
        */
        double minDistOctomap (geometry_msgs::Pose last_pose,geometry_msgs::Pose predicted_pose, octomap::OcTree* octomap, double vx, double vz);
        // Alineación entre la dirección de velocidad evaluada y la dirección del objetivo en el plano XY.
        double calcYawHeading (geometry_msgs::Pose pose, geometry_msgs::Point goal);

        // Alineacion en altura con el objetivo
        double calcZHeading (geometry_msgs::Pose pose, geometry_msgs::Point goal);

        // Modulo del vector distancia entre pose y goal
        double goalDistance(geometry_msgs::Pose pose, geometry_msgs::Point goal);

        //Posición predicha tras ejecutar una trayectoria con velocidad [vx,vy,vz] durante un tiempo [dt]
        geometry_msgs::Pose simubot (double vx, double w, double vz, double dt);

        bool tryOffboard(void);

        bool land(void);

        bool disarm(void);

        void initMarkers(visualization_msgs::Marker *discarded_poses_debug, 
                        visualization_msgs::Marker *casted_rays_markers);

        //void resetMarkersContents();

        void initDwaVisualMsg(tyrion_dwa::DynamicWindowMsg* DWA_visual_msg,
                    const std::array<double, 6>& Vsd, 
                    int filas_eval, int filas_tot);

        void populateRaysVisualMsg(geometry_msgs::Pose predicted_pose, octomap::OcTree *octomap, 
                                double vx, double vz, visualization_msgs::Marker* casted_rays_markers);

};

