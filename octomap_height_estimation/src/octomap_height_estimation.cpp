#include <octomap_height_estimation.h>

HeightEstimator::HeightEstimator(const ros::NodeHandle &nh,
             const ros::NodeHandle &nh_private) : 
                                  nh_(nh), nh_private_(nh_private)                                
{
    // Load parameters from ros server
    // Topics and frames
    nh_.param("pose_topic", pose_topic, std::string("/floam/pose"));
    ROS_INFO("Params loaded");
    // Pose subscriber
    pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>(pose_topic, 10, &HeightEstimator::poseCallback, this);
    // Octomap topic
    octomap_sub = nh_.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, &HeightEstimator::octomapCallback, this);
    //Z floor publisher
    z_floor_pub = nh_.advertise<std_msgs::Float32>("/z_floor", 10);

    //Init
    pose_recieved = false;
    octomap_recieved = false;
}

void HeightEstimator::estimateZ(){
    octomap::point3d origin = octomap::point3d(current_pose.position.x, current_pose.position.y, current_pose.position.z);
    octomap::point3d ray, end;
    //Ray vector coordinates in a robocentric reference
    ray = octomap::point3d(0, 0, -1);
    //Cast ray and check occupancy
    bool occupied = octomap->castRay(origin, ray, end, true, -1.0);
    bool unknown = false;
    if(!occupied){
        unknown = (octomap->search(end) == NULL);
    }
    if(occupied || unknown){ // True if impact an occupied voxel
        auto zc = end.z();
        z_floor_msg.data = zc;
        z_floor_pub.publish(z_floor_msg);
    }else{
        ROS_WARN("Floor is free");
        z_floor_msg.data = 0.0; //TO-DO: What value should it have if it is free
        z_floor_pub.publish(z_floor_msg);
    }
}

void HeightEstimator::loop()
{
    ros::Rate control_rate(20);
    ros::Time last_octomap_time, current_time;
    last_octomap_time = ros::Time::now();

    //Wait until pose and octomap are recieved
    while(ros::ok() && (!pose_recieved || !octomap_recieved)){
        ros::spinOnce();
        control_rate.sleep();
        if(!octomap_recieved){
            ROS_WARN("Waiting to recieve octomap");
        }
        if(!pose_recieved){
            ROS_WARN("Waiting to recieve pose");
        }
    }

    while (ros::ok()){
        ros::spinOnce();
        control_rate.sleep();
        // Retrieve the Octomap from the server
        current_time = ros::Time::now();
	//std::cout << "Last Time " << last_octomap_time.toSec() << std::endl;
	//std::cout << "Current Time " << current_time.toSec() << std::endl;
        //if (current_time.toSec() - last_octomap_time.toSec() > 1.0){
            octomap = getOctomap();
            //last_octomap_time = current_time;
        //}
        //Estimate floor height
        estimateZ();
    }
}

void HeightEstimator::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // TO-DO Make it pose stamped instead of just pose to keep the header
    current_pose = msg->pose;
    pose_recieved = true;
}

/*
    Function that is in charge of retrieving the octomap from the server
    and performing the needed transformations.
*/
void HeightEstimator::octomapCallback(octomap_msgs::Octomap msg){
    last_octomap_msg = msg;
    octomap_recieved = true;
}


octomap::OcTree *HeightEstimator::getOctomap()
{
    return (octomap::OcTree *)octomap_msgs::msgToMap(last_octomap_msg);
}



