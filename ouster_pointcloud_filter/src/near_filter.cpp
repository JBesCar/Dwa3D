#include <iostream>
#include <ros/ros.h>
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
#include <pcl/point_types.h>

#include <pcl/filters/crop_box.h>



class NearFilter{
    private:

        ros::Subscriber lidar_points_subs;
        ros::Publisher filtered_cloud_pub;
        ros::NodeHandle nh;


        sensor_msgs::PointCloud2 filtered_cloud_msgs;

        // Create the filtering object
        pcl::CropBox<pcl::PCLPointCloud2> filter;


        public:
            NearFilter(){
                nh = ros::NodeHandle();
                //TO-DO Parametrize and use arguments

                // Subscribe and advertise topics
                lidar_points_subs = nh.subscribe<sensor_msgs::PointCloud2>("/ouster_raw/points",10,&NearFilter::cloudCallback, this);
                filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/ouster/points",10);

                //Configure the filter
                filter.setMin(Eigen::Vector4f(-0.4, -0.4, -0.25, 1)); //TO-DO: Parametrize with drone size
                filter.setMax(Eigen::Vector4f(0.4, 0.4, 0.25, 1));
                filter.setNegative(true);
            }

            void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &recieved_cloud_msg){
                pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2());
                pcl::PCLPointCloud2::Ptr pcl_cloud_filtered(new pcl::PCLPointCloud2());
                //Convert the PointCloud2 msg to a pcl pointcloud
                pcl_conversions::toPCL(*recieved_cloud_msg, *pcl_cloud);
                //Filter the cloud
                filter.setInputCloud(pcl_cloud);
                filter.filter(*pcl_cloud_filtered);
                //Convert back to PointCloud2 msg and publish 
                pcl_conversions::fromPCL(*pcl_cloud_filtered, filtered_cloud_msgs);
                filtered_cloud_msgs.header.stamp = recieved_cloud_msg->header.stamp;
                filtered_cloud_msgs.header.frame_id = recieved_cloud_msg->header.frame_id;
                filtered_cloud_pub.publish(filtered_cloud_msgs);
            }

};

int main(int argc, char** argv){
    ros::init(argc, argv, "point_cloud_near_filter");
    NearFilter nearFilter;
    ros::spin();
    return 0;
}
