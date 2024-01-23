#include <ros/ros.h>
#include <tf/tf.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#define PCL_NO_PRECOMPILE
//#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/pcl_macros.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <sensor_msgs/Range.h>

#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_datatypes.h>



class tfListener {
    private:

    ros::NodeHandle nh;
    ros::Subscriber subs;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    public:
        tfListener() :  tf_listener_(tf_buffer_, nh){
            nh = ros::NodeHandle();
            subs = nh.subscribe<sensor_msgs::Range>("/roof_range",10,&tfListener::listenCallback,this);
        }

        void listenCallback(sensor_msgs::Range range){
            //TO-DO  Incluir los nombres de los tf como argumentos
            geometry_msgs::TransformStamped transform_lidar_msgs, transform_roof_range_mgsg;
            try{
                transform_roof_range_mgsg = tf_buffer_.lookupTransform("world", "base_link", ros::Time(0));
                transform_lidar_msgs = tf_buffer_.lookupTransform("world", "os0_lidar", ros::Time(0));
            } catch (tf2::TransformException ex){
                ROS_ERROR("%s",ex.what());
            }  
        }


};


int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_listener");
    tfListener listener;
    ros::spin();
    return 0;
}