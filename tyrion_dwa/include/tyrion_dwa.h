#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>


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

const double T = 0.1; //Periodo de control [s]
const double vx_max = 0.3; //Default = 1
const double vz_max = 0.3; //Default = 0.5
const double w_max = PI/4; // 30º --> 15º ??, Default = PI/9
const double radio_dron = 0.4; //[m], Default = 0.5
const double r_search = 1.5; // [m]
const double velodyne_max_range = 10; //Rango máximo de sensor velodyne [m]. Es el mismo definido en VLP-16.urdf.xacro, Default = 10
const double res_azimuth = 90 * PI/180; //Resolución angular [°] en azimuth 30
const double res_elevation = 20 * PI/180; //Resolución angular [°] en elevation 30
const double resolution = 10 * PI/180;
const double paso_v = 0.05; //Resolución de discretización del espacio de búsqueda en xz[m/s], Default = 0.05
const double paso_w = PI/72; // Resolucion de discretizacion del espacio de busqueda en yaw (5º), Default = Pi/36
const double aLin = 1; // Aceleracion lineal máxima [m/ss], Default = 1.0
const double aAng = PI/1.8; // Aceleracion angular maxima [rad/ss] 10º, Default = Pi/1.8
const int cols = 8;
const int filas_tot = ((2*aLin*T/paso_v) + 1)*((2*aAng*T/paso_w) + 1)*((2*aLin*T/paso_v) + 1);


class Dwa3d {
    private:
        ros::NodeHandle nh_, nh_private_;
        ros::Publisher vel_pub, DWA_visual_pub;
        ros::Subscriber pose_sub, state_sub, extended_state_sub, current_vel_sub, plan_sub, octomap_sub;
        ros::Publisher markers_debug_pub, predicted_pose_pub, discarded_poses_pub, vel_visual_pub;
        ros::ServiceClient set_cmd_vel_frame, arming_client, set_mode_client;

        geometry_msgs::Twist empty,cmd, current_vel_msg;
        geometry_msgs::TwistStamped vel_visual_msg;
        std::vector<geometry_msgs::Pose> trajectory;
        geometry_msgs::Pose current_pose;
       
        bool success, executing;

        //MAVROS
        mavros_msgs::SetMode mavros_set_mode;
        mavros_msgs::CommandBool arm_cmd;
        mavros_msgs::State current_state;
        mavros_msgs::ExtendedState current_extended_state;

        // DWA
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        double ALFA, Ky, Kz, BETA, GAMMA;
        double NEAR = 0; // Parametro para momento final de acercamiento al objetivo
        double step, subgoal_step;
        std::array<double,6> Vs = { 0, vx_max, -w_max, w_max, -vz_max, vz_max }; // Espacio de velocidades maximas
        bool lecturaObstaculos = true;
        bool current_vel_recieved = false;
        bool pose_recieved = false;
        bool octomap_recieved = false;
        int goal_i = 1;

        // Map
        ros::ServiceClient octomap_server;
        octomap_msgs::Octomap last_octomap_msg;
        octomap::OcTree* octomap;
        
        int iter_obs; //iter_obs*T = Periodo [s] de actualización de obstáculos = Horizonte temporal en la predicción de posición
        int iter_update;

        // Topics
        std::string cmd_vel_control_topic, ground_truth_topic,
                    cmd_frame;


    public:
        Dwa3d(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
            
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
        double minDistOctomap (geometry_msgs::Pose last_pose,geometry_msgs::Pose predicted_pose, octomap::OcTree* octomap);
        // Alineación entre la dirección de velocidad evaluada y la dirección del objetivo en el plano XY.
        double calcYawHeading (geometry_msgs::Pose pose, geometry_msgs::Point goal);

        // Alineacion en altura con el objetivo
        double calcZHeading (geometry_msgs::Pose pose, geometry_msgs::Point goal);

        // Modulo del vector distancia entre pose y goal
        double goalDistance(geometry_msgs::Pose pose, geometry_msgs::Point goal);

        //Posición predicha tras ejecutar una trayectoria con velocidad [vx,vy,vz] durante un tiempo [dt]
        geometry_msgs::Pose simubot (double vx, double w, double vz, double dt);

        bool isPoseSafe(geometry_msgs::Pose pose_predicted);

        bool tryOffboard(void);

        bool land(void);

        bool disarm(void);

        void queryReplan();
};



/* 
int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_executor");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ROS_INFO("Creating DWA_controller");
    Dwa3d controller(nh, nh_private);
    ROS_INFO("DWA_controller created");
    controller.idle();
    return 0;
}
 */