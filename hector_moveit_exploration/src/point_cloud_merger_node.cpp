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

class CloudMerger {
    private:

    ros::Subscriber lidar_points_subs, roof_measure_subs, floor_measure_subs, odom_subs;
    ros::Publisher virtual_roof_cloud_pub, virtual_floor_cloud_pub, merged_cloud_pub;
    ros::NodeHandle nh;


    sensor_msgs::PointCloud2 lidar_cloud_msgs, virtual_roof_cloud_msgs, virtual_floor_cloud_msgs, merged_cloud_msgs;
    bool recieved_cloud, recieved_roof_range, recieved_floor_range;
    double roof_tile_size, floor_tile_size;
    int points_per_axis;
    std::string lidar_topic, roof_range_topic, floor_range_topic, merged_clouds_topic;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    public:
        CloudMerger() :  tf_listener_(tf_buffer_, nh){
            nh = ros::NodeHandle();
            //TO-DO Parametrize and use arguments
            nh.param("/roof_tile_size", roof_tile_size, 2.0); 
            nh.param("/floor_tile_size", floor_tile_size, 1.0);
            nh.param("/points_per_axis", points_per_axis, 50); 
            nh.param("/lidar_points_topic", lidar_topic, std::string("/velodyne_points"));
            nh.param("/roof_range_topic", roof_range_topic, std::string("/roof_range"));
            nh.param("/floor_range_topic", floor_range_topic, std::string("/sonar_height"));
            nh.param("/merged_cloud_topic", merged_clouds_topic, std::string("merged_clouds"));

            // Subscribe and advertise topics
            lidar_points_subs = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic,10,&CloudMerger::velodyneCloudCallback, this);
            roof_measure_subs = nh.subscribe<sensor_msgs::Range>(roof_range_topic,10,&CloudMerger::roofCallback, this);
            floor_measure_subs = nh.subscribe<sensor_msgs::Range>(floor_range_topic,10,&CloudMerger::floorCallback, this);
            virtual_roof_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("virtual_roof_cloud",10);
            virtual_floor_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("virtual_floor_cloud",10);
            merged_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(merged_clouds_topic,10);
            recieved_cloud = false, recieved_roof_range= false, recieved_floor_range = false;
        }

        void velodyneCloudCallback(sensor_msgs::PointCloud2 cloud){
            lidar_cloud_msgs = cloud;
            recieved_cloud = true;
            if(recieved_cloud && recieved_roof_range && recieved_floor_range){
                mergeClouds(lidar_cloud_msgs, virtual_roof_cloud_msgs, virtual_floor_cloud_msgs);
            }
        }


        /*
            Creates a virtual poincloud to emulate the roof restriction

            It takes the range measurement to 
            place the cloud at the right height
        */
        void roofCallback(sensor_msgs::Range roof_range){
            double z0;
            auto range = roof_range.range;
            recieved_roof_range = true;

            //TO-DO: Create a virtual pointcloud for a piece of the roof

            // Populate message fields
            const uint32_t POINT_STEP = 48;
            virtual_roof_cloud_msgs.fields.resize(9);
            virtual_roof_cloud_msgs.fields[0].name = "x";
            virtual_roof_cloud_msgs.fields[0].offset = 0;
            virtual_roof_cloud_msgs.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
            virtual_roof_cloud_msgs.fields[0].count = 1;
            virtual_roof_cloud_msgs.fields[1].name = "y";
            virtual_roof_cloud_msgs.fields[1].offset = 4;
            virtual_roof_cloud_msgs.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
            virtual_roof_cloud_msgs.fields[1].count = 1;
            virtual_roof_cloud_msgs.fields[2].name = "z";
            virtual_roof_cloud_msgs.fields[2].offset = 8;
            virtual_roof_cloud_msgs.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
            virtual_roof_cloud_msgs.fields[2].count = 1;
            virtual_roof_cloud_msgs.fields[3].name = "intensity";
            virtual_roof_cloud_msgs.fields[3].offset = 16;
            virtual_roof_cloud_msgs.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
            virtual_roof_cloud_msgs.fields[3].count = 1;
            virtual_roof_cloud_msgs.fields[4].name = "t";
            virtual_roof_cloud_msgs.fields[4].offset = 20;
            virtual_roof_cloud_msgs.fields[4].datatype = sensor_msgs::PointField::UINT32;
            virtual_roof_cloud_msgs.fields[4].count = 1;
            virtual_roof_cloud_msgs.fields[5].name = "reflectivity";
            virtual_roof_cloud_msgs.fields[5].offset = 24;
            virtual_roof_cloud_msgs.fields[5].datatype = sensor_msgs::PointField::UINT16;
            virtual_roof_cloud_msgs.fields[5].count = 1;
            virtual_roof_cloud_msgs.fields[6].name = "ring";
            virtual_roof_cloud_msgs.fields[6].offset = 26;
            virtual_roof_cloud_msgs.fields[6].datatype = sensor_msgs::PointField::UINT16;
            virtual_roof_cloud_msgs.fields[6].count = 1;
            virtual_roof_cloud_msgs.fields[7].name = "ambient";
            virtual_roof_cloud_msgs.fields[7].offset = 28;
            virtual_roof_cloud_msgs.fields[7].datatype = sensor_msgs::PointField::UINT16;
            virtual_roof_cloud_msgs.fields[7].count = 1;
            virtual_roof_cloud_msgs.fields[8].name = "range";
            virtual_roof_cloud_msgs.fields[8].offset = 32;
            virtual_roof_cloud_msgs.fields[8].datatype = sensor_msgs::PointField::UINT32;
            virtual_roof_cloud_msgs.fields[8].count = 1;

            virtual_roof_cloud_msgs.data.resize((points_per_axis)  * (points_per_axis) * POINT_STEP);


            uint8_t *ptr = virtual_roof_cloud_msgs.data.data();
            double resolution = roof_tile_size/points_per_axis;
            double center = roof_tile_size/2;
            double z_cloud = range;
            //double z_cloud = 2.5;
            // TO-DO Centre the cloud with respect to the drone
            for(float i = 0; i < points_per_axis ; i++){
                auto x_coord = i * resolution - center;
                for(int j = 0; j < points_per_axis; j++){
                    auto y_coord = j * resolution - center;
                    *((float*)(ptr + 0)) = x_coord;
                    *((float*)(ptr + 4)) = y_coord;
                    *((float*)(ptr + 8)) = z_cloud;
                    *((float*)(ptr + 16)) = 2; //Intensity
                    *((float*)(ptr + 20)) = 0; //t
                    *((uint16_t*)(ptr + 24)) = 0; // reflectivity
		    *((uint16_t*)(ptr + 26)) = 0; //Ring
		    *((float*)(ptr + 28)) = 0; // ambient
		    *((float*)(ptr + 32)) = 0; // range
                    ptr += POINT_STEP;
                }
            }

            virtual_roof_cloud_msgs.header.frame_id = "odom"; //sonar_roof_link
            virtual_roof_cloud_msgs.header.stamp = ros::Time::now();
            virtual_roof_cloud_msgs.point_step = POINT_STEP;
            virtual_roof_cloud_msgs.width = virtual_roof_cloud_msgs.data.size() / POINT_STEP;
            virtual_roof_cloud_msgs.height = 1;
            virtual_roof_cloud_msgs.row_step = virtual_roof_cloud_msgs.data.size();
            virtual_roof_cloud_msgs.is_dense = true;

            
            // //Convert to PointCloud2
            // pcl::toROSMsg(cloud, virtual_roof_cloud_msgs);
            // //virtual_roof_cloud_msgs.data.resize(virtual_roof_cloud_msgs.data.data())
            // virtual_roof_cloud_msgs.header.frame_id = "base_link"; //sonar_roof_link
            // virtual_roof_cloud_msgs.header.stamp = ros::Time::now();

            
            virtual_roof_cloud_pub.publish(virtual_roof_cloud_msgs);

            //Merge the clouds if both are available
            if(recieved_cloud && recieved_roof_range && recieved_floor_range){
                mergeClouds(lidar_cloud_msgs, virtual_roof_cloud_msgs, virtual_floor_cloud_msgs);
            }
        }



        /*
            Creates a virtual poincloud to emulate the floor restriction

            It takes the range measurement to 
            place the cloud at the right height
        */
        void floorCallback(sensor_msgs::Range floor_range){
            double z0;
            auto range = floor_range.range;
            //int verticalRangeCount = round(points_per_axis/floor_tile_size);
            recieved_floor_range = true;

            //TO-DO: Create a virtual pointcloud for a piece of the roof

            // Populate message fields
            const uint32_t POINT_STEP = 48;
            virtual_floor_cloud_msgs.fields.resize(9);
            virtual_floor_cloud_msgs.fields[0].name = "x";
            virtual_floor_cloud_msgs.fields[0].offset = 0;
            virtual_floor_cloud_msgs.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
            virtual_floor_cloud_msgs.fields[0].count = 1;
            virtual_floor_cloud_msgs.fields[1].name = "y";
            virtual_floor_cloud_msgs.fields[1].offset = 4;
            virtual_floor_cloud_msgs.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
            virtual_floor_cloud_msgs.fields[1].count = 1;
            virtual_floor_cloud_msgs.fields[2].name = "z";
            virtual_floor_cloud_msgs.fields[2].offset = 8;
            virtual_floor_cloud_msgs.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
            virtual_floor_cloud_msgs.fields[2].count = 1;
            virtual_floor_cloud_msgs.fields[3].name = "intensity";
            virtual_floor_cloud_msgs.fields[3].offset = 16;
            virtual_floor_cloud_msgs.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
            virtual_floor_cloud_msgs.fields[3].count = 1;
            virtual_floor_cloud_msgs.fields[4].name = "t";
            virtual_floor_cloud_msgs.fields[4].offset = 20;
            virtual_floor_cloud_msgs.fields[4].datatype = sensor_msgs::PointField::UINT32;
            virtual_floor_cloud_msgs.fields[4].count = 1;
            virtual_floor_cloud_msgs.fields[5].name = "reflectivity";
            virtual_floor_cloud_msgs.fields[5].offset = 24;
            virtual_floor_cloud_msgs.fields[5].datatype = sensor_msgs::PointField::UINT16;
            virtual_floor_cloud_msgs.fields[5].count = 1;
            virtual_floor_cloud_msgs.fields[6].name = "ring";
            virtual_floor_cloud_msgs.fields[6].offset = 26;
            virtual_floor_cloud_msgs.fields[6].datatype = sensor_msgs::PointField::UINT16;
            virtual_floor_cloud_msgs.fields[6].count = 1;
            virtual_floor_cloud_msgs.fields[7].name = "ambient";
            virtual_floor_cloud_msgs.fields[7].offset = 28;
            virtual_floor_cloud_msgs.fields[7].datatype = sensor_msgs::PointField::UINT16;
            virtual_floor_cloud_msgs.fields[7].count = 1;
            virtual_floor_cloud_msgs.fields[8].name = "range";
            virtual_floor_cloud_msgs.fields[8].offset = 32;
            virtual_floor_cloud_msgs.fields[8].datatype = sensor_msgs::PointField::UINT32;
            virtual_floor_cloud_msgs.fields[8].count = 1;
            //virtual_floor_cloud_msgs.data.resize((2*verticalRangeCount+1)  * (2*rangeCount+1) * POINT_STEP);
            virtual_floor_cloud_msgs.data.resize((points_per_axis)  * (points_per_axis) * POINT_STEP);

            uint8_t *ptr = virtual_floor_cloud_msgs.data.data();
            //Create a plane at height z and fill it with points
            double resolution = floor_tile_size/points_per_axis;
            double center = floor_tile_size/2;
            double z_cloud = -range;
            // TO-DO Centre the cloud with respect to the drone
            for(float i = 0; i < points_per_axis ; i++){
                auto x_coord = i * resolution - center;
                for(int j = 0; j < points_per_axis; j++){
                    auto y_coord = j * resolution - center;
                    *((float*)(ptr + 0)) = x_coord;
                    *((float*)(ptr + 4)) = y_coord;
                    *((float*)(ptr + 8)) = z_cloud;
                    *((float*)(ptr + 16)) = 2; //Intensity
                    *((float*)(ptr + 20)) = 0.0; //t
                    *((uint16_t*)(ptr + 24)) = 0; // reflectivity
		    *((uint16_t*)(ptr + 26)) = 0; //Ring
		    *((float*)(ptr + 28)) = 0; // ambient
		    *((float*)(ptr + 32)) = 0; // range
                    ptr += POINT_STEP;
                }
            }
            virtual_floor_cloud_msgs.header.frame_id = "base_link"; //TO-DO: Check this tf
            virtual_floor_cloud_msgs.header.stamp = ros::Time::now();
            virtual_floor_cloud_msgs.point_step = POINT_STEP;
            virtual_floor_cloud_msgs.width = virtual_floor_cloud_msgs.data.size() / POINT_STEP;
            virtual_floor_cloud_msgs.height = 1;
            virtual_floor_cloud_msgs.row_step = virtual_floor_cloud_msgs.data.size();
            virtual_floor_cloud_msgs.is_dense = true;


            virtual_floor_cloud_pub.publish(virtual_floor_cloud_msgs);

            //Merge the clouds if both are available
            if(recieved_cloud && recieved_roof_range && recieved_floor_range){
                mergeClouds(lidar_cloud_msgs, virtual_roof_cloud_msgs, virtual_floor_cloud_msgs);
            }
        }


        void mergeClouds(sensor_msgs::PointCloud2 cloud_lidar, sensor_msgs::PointCloud2 cloud_roof, sensor_msgs::PointCloud2 cloud_floor){
            //TO-DO  Incluir los nombres de los tf como argumentos
            recieved_cloud = false, recieved_roof_range= false, recieved_floor_range = false;
            geometry_msgs::TransformStamped transform_lidar_msgs, transform_roof_range_mgsg, transform_floor_range_mgsg;
            sensor_msgs::PointCloud2 cloud_lidar_trans, cloud_roof_trans, cloud_floor_trans;
            pcl::PCLPointCloud2 pcl_lidar, pcl_roof, pcl_floor, pcl_merged;
            try{
                transform_roof_range_mgsg = tf_buffer_.lookupTransform("odom_ned", cloud_roof.header.frame_id, ros::Time(0));
                transform_lidar_msgs = tf_buffer_.lookupTransform("odom_ned", cloud_lidar.header.frame_id, ros::Time(0));
                transform_floor_range_mgsg = tf_buffer_.lookupTransform("odom_ned", cloud_floor.header.frame_id, ros::Time(0));
                //transform_lidar_msgs = tf_buffer_.lookupTransform("base_link", "os0_lidar", ros::Time(0));
            } catch (tf2::TransformException ex){
                ROS_ERROR("%s",ex.what());
            }
            tf2::doTransform (cloud_lidar, cloud_lidar_trans, transform_lidar_msgs);
            tf2::doTransform (cloud_floor, cloud_floor_trans, transform_floor_range_mgsg);
            tf2::doTransform (cloud_roof, cloud_roof_trans, transform_roof_range_mgsg);
            
            pcl_conversions::toPCL(cloud_lidar_trans,pcl_lidar);
            pcl_conversions::toPCL(cloud_roof_trans, pcl_roof);
            pcl_conversions::toPCL(cloud_floor_trans, pcl_floor);

            pcl::concatenatePointCloud(pcl_lidar, pcl_roof, pcl_merged);
	    pcl::concatenatePointCloud(pcl_merged, pcl_floor, pcl_merged);
	
            pcl_conversions::fromPCL(pcl_merged,merged_cloud_msgs);
            
	    
            merged_cloud_msgs.header.frame_id = "odom_ned"; //base_link
            merged_cloud_msgs.header.stamp = ros::Time::now();
            merged_cloud_pub.publish(merged_cloud_msgs);
        }


};



int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_merger");
    CloudMerger cloudMerger;
    ros::spin();
    return 0;
}





