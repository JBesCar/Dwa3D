#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>

#include <cmath>
#define _USE_MATH_DEFINES

#define MAX_SPEED 1.5
#define EPSILON 1e-4


//OctoMap
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_server/OctomapServer.h>


class HeightEstimator {
    private:
        ros::NodeHandle nh_, nh_private_;
        ros::Publisher z_floor_pub;
        ros::Subscriber pose_sub, octomap_sub;

        geometry_msgs::Pose current_pose;
        bool pose_recieved, octomap_recieved;
        std_msgs::Float32 z_floor_msg;

        // Map
        octomap_msgs::Octomap last_octomap_msg;
        octomap::OcTree* octomap;
        
        // Topics and frames
        std::string pose_topic;


    public:
        HeightEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

        void estimateZ();

        void loop();

        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg);
        
        /*
            Function that is in charge of retrieving the octomap from the server
            and performing the needed transformations.
        */
        void octomapCallback(octomap_msgs::Octomap msg);
        octomap::OcTree* getOctomap();


};

