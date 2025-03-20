// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>


//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"
#include "odomEstimationClass.h"

OdomEstimationClass odomEstimation;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
sensor_msgs::Imu imu_msg;
float z_floor_curr = 0, z_floor_prev = 0;
lidar::Lidar lidar_param;
tf::Transform initial_tf, lidar_to_body_tf;

ros::Publisher pubLaserOdometry, posePub;
ros::Subscriber imu_subscriber, floorDist_subscriber;
std::string base_link_frame, gt_frame, pose_pub_topic;
bool init_with_optitrack;
bool initial_tf_recieved = false;
bool initial_range_recieved = false;


void velodyneSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudSurfBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}
void velodyneEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudEdgeBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg){ 
    imu_msg = *msg;
    initial_tf_recieved = true;
}

//JBES: MMSE Z estimation
void rangeCallback(const sensor_msgs::Range::ConstPtr &msg){
    //z_floor_prev = z_floor_curr;
    z_floor_curr = msg->range;
    if(!initial_range_recieved){
        z_floor_prev = z_floor_curr;
        initial_range_recieved = true;
    }
}

bool is_odom_inited = false;
double total_time =0;
int total_frame=0;
void odom_estimation(){
    //JBES: INIT WITH IMU   
    if(!init_with_optitrack){
        do{
		std::cout << "Waiting IMU" << std::endl;
		ros::spinOnce();
	}while(!initial_tf_recieved);
	std::cout << "Init with IMU" << std::endl;
    //Get initial orientation
    initial_tf.setOrigin(tf::Vector3(0, 0, 0.14));
    tf::Quaternion q(imu_msg.orientation.x,imu_msg.orientation.y,
                    imu_msg.orientation.z,imu_msg.orientation.w);
	tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	//Force yaw = 0
	q.setRPY(roll, pitch, 0); 
        initial_tf.setRotation(q);
	std::cout << "Orientation Inited with IMU" << std::endl;
        std::cout << "QX: " << initial_tf.getRotation().x() << std::endl;
        std::cout << "QY: " << initial_tf.getRotation().y() << std::endl;
	std::cout << "QZ: " << initial_tf.getRotation().z() << std::endl;
	std::cout << "QW: " << initial_tf.getRotation().w() << std::endl;
    }


    //JBES: MMSE Z estimation
    static float z_floam_curr = 0, z_floam_prev = 0;
    static float z_est_prev = 0, z_est_curr = 0;

    while(1){
        if(!pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty()){

            //read data
            mutex_lock.lock();
            if(!pointCloudSurfBuf.empty() && (pointCloudSurfBuf.front()->header.stamp.toSec()<pointCloudEdgeBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period)){
                pointCloudSurfBuf.pop();
                ROS_WARN_ONCE("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock.unlock();
                continue;  
            }

            if(!pointCloudEdgeBuf.empty() && (pointCloudEdgeBuf.front()->header.stamp.toSec()<pointCloudSurfBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period)){
                pointCloudEdgeBuf.pop();
                ROS_WARN_ONCE("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock.unlock();
                continue;  
            }
            //if time aligned 

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_edge_in);
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_surf_in);
            ros::Time pointcloud_time = (pointCloudSurfBuf.front())->header.stamp;
            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();
            mutex_lock.unlock();

            if(is_odom_inited == false){
                odomEstimation.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
                is_odom_inited = true;
                ROS_INFO("odom inited");
                //JBES
                z_floam_prev = odomEstimation.odom.translation().z();
                z_est_prev = z_floam_prev;
            }else{
                std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();
                odomEstimation.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in);
                end = std::chrono::system_clock::now();
                std::chrono::duration<float> elapsed_seconds = end - start;
                total_frame++;
                float time_temp = elapsed_seconds.count() * 1000;
                total_time+=time_temp;
                //ROS_INFO("average odom estimation time %f ms \n \n", total_time/total_frame);
            }



            Eigen::Quaterniond q_current(odomEstimation.odom.rotation());
            //q_current.normalize();
            Eigen::Vector3d t_current = odomEstimation.odom.translation();
            //JBES: MMSE Z estimation
            z_floam_curr = t_current.z();
            //Correct lidar1D measurement with pitch
            {
            tf::Quaternion q(imu_msg.orientation.x,imu_msg.orientation.y,
                            imu_msg.orientation.z,imu_msg.orientation.w);
	        tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            //z_floor_curr *= cos(pitch);
            }
	    float delta_z_floor, delta_z_floam;
	    delta_z_floam = z_floam_curr - z_floam_prev;
   	    delta_z_floor = z_floor_curr - z_floor_prev;
	    z_est_curr = z_est_prev + 0.3 * (delta_z_floam) + 0.7 * (delta_z_floor);	
            z_floam_prev = z_floam_curr;
            z_floor_prev = z_floor_curr;
            z_est_prev = z_est_curr;
            //Update z with the estimation
            //t_current = Eigen::Vector3d(t_current.x(), t_current.y(), z_est_curr);
            //odomEstimation.odom.translation() = t_current;
            //Prepare the TFs and msgs to publish
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(t_current.x(), t_current.y(), t_current.z()) );
            tf::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
            transform.setRotation(q);
            //initial_tf.setOrigin(tf::Vector3(90.0, 0.0, 0.0));
            //initial_tf.setRotation(tf::Quaternion(0, 0, 0, 1));
            transform = initial_tf * transform;
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", base_link_frame));

            // publish odometry
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "odom";
            laserOdometry.child_frame_id = base_link_frame;
            laserOdometry.header.stamp = pointcloud_time;
            laserOdometry.pose.pose.orientation.x = q_current.x();
            laserOdometry.pose.pose.orientation.y = q_current.y();
            laserOdometry.pose.pose.orientation.z = q_current.z();
            laserOdometry.pose.pose.orientation.w = q_current.w();
            laserOdometry.pose.pose.position.x = t_current.x();
            laserOdometry.pose.pose.position.y = t_current.y();
            laserOdometry.pose.pose.position.z = t_current.z();
            pubLaserOdometry.publish(laserOdometry);

            //JBES: publish pose_msg
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "odom";
            pose.header.stamp = pointcloud_time;
            tf::Quaternion q_tf = transform.getRotation();
            tf::Vector3 t_tf = transform.getOrigin();
            pose.pose.orientation.x = q_tf.getX();
            pose.pose.orientation.y = q_tf.getY();
            pose.pose.orientation.z = q_tf.getZ();
            pose.pose.orientation.w = q_tf.getW();
            pose.pose.position.x = t_tf.getX();
            pose.pose.position.y = t_tf.getY();
            pose.pose.position.z = t_tf.getZ();
            posePub.publish(pose);

        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(40);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;
    double map_resolution = 0.4;
    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);
    nh.getParam("/map_resolution", map_resolution);
    
    //JBES Changes:: Add topics and tf names
    nh_private.param("/floam_odom_estimation_node/base_link_frame",base_link_frame ,std::string("base_link"));
    nh_private.param("/floam_odom_estimation_node/gt_frame", gt_frame, std::string("base_link_gt"));
    nh_private.param("/floam_odom_estimation_node/pose_pub_topic", pose_pub_topic, std::string("/floam/pose"));
    nh_private.param("/floam_odom_estimation_node/init_with_optitrack", init_with_optitrack, true);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    //JBES Changes: Init with inital position
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_(tf_buffer_, nh);
    geometry_msgs::TransformStamped initial_tf_msg, lidar_to_body_tf_msg;
    //Eigen::Isometry3d initial_position;
    
    bool lidar_to_body_tf_recieved = false;

    imu_subscriber = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 100, imuCallback);


    //Get base_link <-> LiDAR TF
    do{
        try{
            lidar_to_body_tf_msg = tf_buffer_.lookupTransform("os_lidar", "base_link", ros::Time(0));
            lidar_to_body_tf_recieved = true;
        }catch (tf2::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
    }while(!lidar_to_body_tf_recieved);
    tf::transformMsgToTF(lidar_to_body_tf_msg.transform, lidar_to_body_tf);
    
    if(init_with_optitrack){
        //Get initial optitrack pose
        do{
            try{
                initial_tf_msg = tf_buffer_.lookupTransform("odom", gt_frame, ros::Time(0));
                initial_tf_recieved = true;
            }catch (tf2::TransformException ex){
                ROS_ERROR("%s",ex.what());
            }
        }while(!initial_tf_recieved);
        tf::transformMsgToTF(initial_tf_msg.transform, initial_tf);
        //tf::transformTFToEigen(initial_tf,initial_position);
     }
    
    odomEstimation.init(lidar_param, map_resolution);//initial_position 
    ros::Subscriber subEdgeLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100, velodyneEdgeHandler);
    ros::Subscriber subSurfLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100, velodyneSurfHandler);
    if(!init_with_optitrack){
    	imu_subscriber = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 100, imuCallback);
    }
    floorDist_subscriber = nh.subscribe<sensor_msgs::Range>("/mavros/distance_sensor/mini_tf_pub", 100, rangeCallback);
    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    posePub = nh.advertise<geometry_msgs::PoseStamped>(pose_pub_topic, 10);
    std::thread odom_estimation_process{odom_estimation};

    ros::spin();

    return 0;
}

