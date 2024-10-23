#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sstream>
#include <string>
#include <iostream>
#include <fstream>


//Publishers and subscribers
ros::Publisher goal_publisher;
ros::Subscriber pose_subscriber;

//Msgs
geometry_msgs::PoseStamped current_pose, current_goal;

//Point3D
typedef struct{
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
}Point3D;

// Mission
typedef struct {
    unsigned int Count = 0 ;
    Point3D Waypoint[100];
} Mission3D;

Mission3D mission;                 
unsigned int Nwaypoint = 0 ;  
bool pose_recieved = false;

std::string line;
std::ifstream infile;

//Params
std::string mission_file;
std::string pose_topic;
std::string goal_topic;
float threshold_distance;

//Local function and prototypes
void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

int main(int argc, char** argv){
    ros::init(argc, argv, "mission_sender");
    ros::NodeHandle nh;
    //Read params
    nh.param("/mission_file", mission_file, std::string("/home/jetson_orin/mission.txt"));
    nh.param("/pose_topic", pose_topic, std::string("/floam/pose"));
    nh.param("/goal_topic", goal_topic, std::string("/vrpn_client_node/goal_optitrack/pose"));
    nh.param("/threshold_distance", threshold_distance, float(4.0));

    // Subscribe and advertise topics
    goal_publisher = nh.advertise<geometry_msgs::PoseStamped>(goal_topic, 10);
    pose_subscriber = nh.subscribe<geometry_msgs::PoseStamped>(pose_topic, 10, poseCallback);

    // Reading the mission file
    infile.open(mission_file);
    while (!infile.eof() & ros::ok()) {
        std::getline (infile, line) ;
        try {			
            std::string::size_type sz;
            float coord_x, coord_y, coord_z; 
            coord_x = std::stof (line,&sz);     // X
            line = line.substr(sz) ;
            coord_y = std::stof (line,&sz);     // Y
            line = line.substr(sz);
            coord_z = std::stof (line,&sz);     // Z
            mission.Waypoint[Nwaypoint].x = coord_x ;
            mission.Waypoint[Nwaypoint].y = coord_y ;
            mission.Waypoint[Nwaypoint].z = coord_z ;
            ROS_INFO("Waypoint X = %f, Y = %f, Z = %f", coord_x, coord_y, coord_z);
            mission.Count ++ ;
            
            Nwaypoint ++ ;
              
        }catch (const std::exception &e){
            std::cerr << e.what() << std::endl  ;
        }  
    }
    Nwaypoint = 0 ;

    //Wait until pose is recieved
    ros::Rate wait_rate(0.5);
    while (ros::ok() && !pose_recieved)
    {
        ROS_WARN("WAITING TO RECIEVE POSE");
        ros::spinOnce();
        wait_rate.sleep();
    }
    

    //Configure goal msg
    current_goal.header.frame_id = "odom";
    //Populate goal mgs
    current_goal.pose.position.x = mission.Waypoint[Nwaypoint].x;
    current_goal.pose.position.y = mission.Waypoint[Nwaypoint].y;
    current_goal.pose.position.z = mission.Waypoint[Nwaypoint].z;

    float distance, diff_x, diff_y, diff_z;

    //Send goals when it is needed
    ros::Rate rate(5.0);
    while(ros::ok() &&  Nwaypoint < mission.Count - 1){
        //Check distance to next goal
        diff_x = current_pose.pose.position.x - current_goal.pose.position.x;
        diff_y = current_pose.pose.position.y - current_goal.pose.position.y;
        diff_z = current_pose.pose.position.z - current_goal.pose.position.z;
        distance = sqrt(diff_x*diff_x + diff_y*diff_y + diff_z*diff_z);
        //ROS_INFO("Distance to next goal: %f", distance);
        if(distance < threshold_distance){
            Nwaypoint++;
            //Populate goal mgs
            current_goal.pose.position.x = mission.Waypoint[Nwaypoint].x;
            current_goal.pose.position.y = mission.Waypoint[Nwaypoint].y;
            current_goal.pose.position.z = mission.Waypoint[Nwaypoint].z;
            ROS_INFO("Waypoint X = %f, Y = %f, Z = %f", 
                    current_goal.pose.position.x, 
                    current_goal.pose.position.y, 
                    current_goal.pose.position.z);
        }
        //Publish current goal
        goal_publisher.publish(current_goal);
        ros::spinOnce();
        rate.sleep();
    }
}

void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg){
    current_pose = *msg;
    pose_recieved = true;
}
