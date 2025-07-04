#include <ros/ros.h>
#include <tf/tf.h>


#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>


#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/tf.h>

class poseTfBroadcaster {
    private:

    ros::NodeHandle nh;
    ros::Subscriber subs;
    ros::Publisher pub;
    geometry_msgs::PoseStamped recieved_pose, transformed_pose;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_; 

    public:
        poseTfBroadcaster() :  tf_listener_(tf_buffer_, nh){
            nh = ros::NodeHandle();
            subs = nh.subscribe<nav_msgs::Odometry>("/tyrion/mavros/odometry/out",10,&poseTfBroadcaster::odomCallback,this);
 	        // pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",10);
        }

        void odomCallback(nav_msgs::Odometry recieved_odom){
            //TO-DO  Incluir los nombres de los tf como argumentos
            geometry_msgs::TransformStamped tf_odom_odomNED;
            try{
                tf_odom_odomNED = tf_buffer_.lookupTransform("odom", "odom", ros::Time(0));
            } catch (tf2::TransformException ex){
                ROS_ERROR("%s",ex.what());
            }
            recieved_pose.pose = recieved_odom.pose.pose;
            tf2::doTransform(recieved_pose, transformed_pose, tf_odom_odomNED);
            transformed_pose.header.frame_id="odom_ned";
            transformed_pose.header.stamp=recieved_pose.header.stamp;
        }

	void timedPubCallback(void){
        transformed_pose.header.stamp = ros::Time::now();	    
        //pub.publish(transformed_pose);
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "tyrion/base_link";
        transformStamped.transform.translation.x = transformed_pose.pose.position.x;
        transformStamped.transform.translation.y = transformed_pose.pose.position.y;
        transformStamped.transform.translation.z = transformed_pose.pose.position.z;
        transformStamped.transform.rotation.x = transformed_pose.pose.orientation.x;
        transformStamped.transform.rotation.y = transformed_pose.pose.orientation.y;
        transformStamped.transform.rotation.z = transformed_pose.pose.orientation.z;
        transformStamped.transform.rotation.w = transformed_pose.pose.orientation.w;
        br.sendTransform(transformStamped);    
	}
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_tf_broadcaster");
    poseTfBroadcaster pose_tf_broadcaster;
    ros::Rate loop_rate(40);
    while (ros::ok()){
      pose_tf_broadcaster.timedPubCallback();
      ros::spinOnce();
      loop_rate.sleep();
    }

    return 0;
}
