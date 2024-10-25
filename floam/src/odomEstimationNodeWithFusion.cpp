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
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>


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
float z_range = 0;
lidar::Lidar lidar_param;
tf::Transform initial_tf, lidar_to_body_tf;

ros::Publisher pubLaserOdometry, posePub, zFloamPub, zEstPub, 
                zPredPub, oREstPub, oLEstPub, D2MahalanobisPub;
ros::Subscriber imu_subscriber, floorDist_subscriber, velocity_subscriber;
std::string base_link_frame, gt_frame, pose_pub_topic;
bool init_with_optitrack;
bool initial_tf_recieved = false;
bool initial_range_recieved = false;

//Altitude fusion
Eigen::Matrix<float, 3, 1> x_pred(3,1), x_est(3,1); //4x1
Eigen::Matrix<float, 3, 3> A(3,3), P_pred(3,3), P_est(3,3), Q(3,3); //4x4
Eigen::Matrix<float, 3, 1> B(3,1); //4x1
Eigen::Matrix<float, 2, 2> R(2,2), S(2,2); //2x2
Eigen::Matrix<float, 2, 1> z(2,1), y(2,1); //2x1
Eigen::Matrix<float, 2, 3> H(2,3), H_off(2,3); //2x4
Eigen::Matrix<float, 3, 2> K(3,2); //4x2
float u = 0.0; //v_z ref
Eigen::Matrix<float, 3, 3> I(3,3); //4x4 
float delta_t = 0.1; //20ms

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
    z_range = msg->range;
    if(!initial_range_recieved){
        initial_range_recieved = true;
    }
}

//JBES: Z Velocity command for altitude fusion
void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg){
    u = msg->twist.linear.z;
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
        initial_tf.setOrigin(tf::Vector3(0, 0, 0.14)); //TO-DO Use body to lidar tf
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

    //JBES: Wait until first range is recieved to init properly z vector
    do{
        ROS_WARN("Waiting to recieve range");
        ros::spinOnce();
    }while(!initial_range_recieved);


    //JBES: Sensor Fusion Init
    x_est << z_range, 0.0, 0.0;

    P_est << 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0,
             0.0, 0.0, 0.0;

    I << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;

    R << 0.001, 0.0,
         0.0, 0.1;

    Q << 0.1, 0.0, 0.0,
         0.0, 0.001, 0.0,
         0.0, 0.0, 0.001;

    H << 1.0,-1.0, 0.0,
         1.0, 0.0, -1.0;

    H_off << 0.0, -1.0, 0.0,
            1.0, 0.0, -1.0;

    A << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0,

    B << delta_t, 0.0, 0.0;
    static float z_floam = 0;
    float D2_mahalanobis = 0.0;
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


            //JBES: Measurement treatment for posterior fusion 
            Eigen::Quaterniond q_current(odomEstimation.odom.rotation());
            Eigen::Vector3d t_current = odomEstimation.odom.translation();            
            z_floam = t_current.z() + 0.14;

            //Correct lidar1D measurement with pitch
            {
            tf::Quaternion q(imu_msg.orientation.x,imu_msg.orientation.y,
                            imu_msg.orientation.z,imu_msg.orientation.w);
	        tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            z_range *= cos(pitch);
            }

            z << z_range,
                 z_floam;
            std::cout << "z: " << z << std::endl;
            
            //JBES: Altitude fusion prediction step
            x_pred = A * x_est + B * u;
            std::cout << "u: " << u << std::endl;
            std::cout << "X Predicted: " << x_pred << std::endl;
            P_pred = A * P_est * A.transpose() + Q;
            //std::cout << "P Predicted: " << P_pred << std::endl;
            //JBES: Correction step
            y = z - H * x_pred;
            std::cout << "y: " << y << std::endl;
            S = H * P_pred * H.transpose() + R;
            //JBES: Rangefinder inconsistency detection
            D2_mahalanobis = y.transpose() * S.inverse() * y;
            if(D2_mahalanobis > 0.02){ //p_value = 0.9
                K = H_off.transpose() * (H_off * H_off.transpose()).inverse();
                P_est = (I - K * H_off) * P_pred;
                ROS_WARN("Rangefinder not reliable");
            }else{
                //std::cout << "S: " << S << std::endl;
                K = P_pred * H.transpose() * S.inverse();
                std::cout << "K: " << K << std::endl;
                std::cout << "X Estimated: " << x_est << std::endl;
                P_est = (I - K * H) * P_pred;
            }
            x_est = x_est + K * y;
            
            //std::cout << "P estimated: " << P_est << std::endl;
            //Update z with the estimation
            //t_current = Eigen::Vector3d(t_current.x(), t_current.y(), x_est(0));
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


            //Debug
            std_msgs::Float32 z_debug;
            z_debug.data = x_est(0);
            zEstPub.publish(z_debug);
            z_debug.data = t_tf.getZ();
            zFloamPub.publish(z_debug);
            z_debug.data = x_pred(0);
            zPredPub.publish(z_debug);
            z_debug.data = x_est(1);
            oREstPub.publish(z_debug);
            z_debug.data = x_pred(2);
            oLEstPub.publish(z_debug);
            z_debug.data = D2_mahalanobis;
            D2MahalanobisPub.publish(z_debug);
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
    nh_private.param("/base_link_frame",base_link_frame ,std::string("base_link"));
    nh_private.param("/gt_frame", gt_frame, std::string("base_link_gt"));
    nh_private.param("/pose_pub_topic", pose_pub_topic, std::string("/floam/pose"));
    nh_private.param("/floam_odom_estimation_with_altitude_fusion_node/init_with_optitrack", init_with_optitrack, true);

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
    velocity_subscriber = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_body", 100, velocityCallback);
    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    posePub = nh.advertise<geometry_msgs::PoseStamped>(pose_pub_topic, 10);
    //Debug Altitude Fusion
    zFloamPub = nh.advertise<std_msgs::Float32>("/z_floam_est", 10); 
    zEstPub = nh.advertise<std_msgs::Float32>("/z_est_pub", 10);
    zPredPub = nh.advertise<std_msgs::Float32>("/z_pred_pub", 10);
    oREstPub = nh.advertise<std_msgs::Float32>("/oR_est_pub", 10);
    oLEstPub = nh.advertise<std_msgs::Float32>("/oL_est_pub", 10);
    D2MahalanobisPub = nh.advertise<std_msgs::Float32>("/D2Mahalanobis", 10);
    std::thread odom_estimation_process{odom_estimation};

    ros::spin();

    return 0;
}

