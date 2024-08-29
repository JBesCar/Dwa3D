#include <naive_planning.h>

NaivePlanner::NaivePlanner(ros::NodeHandle& nh)
{
    odom_received = false;
    trajectory_received = false;
    goal_recieved = false;
    nh.param("/odom_topic", odom_topic, default_odom_topic);
    nh.param("/goal_topic", goal_topic, default_goal_topic);
    nh.param("/waypoints_topic", waypoints_topic, default_waypoints_topic);

    base_sub = nh.subscribe<geometry_msgs::PoseStamped>(odom_topic,10,&NaivePlanner::poseCallback,this);
    goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(goal_topic,10,&NaivePlanner::goalCallback,this);
    waypoints_pub = nh.advertise<geometry_msgs::PoseArray>(waypoints_topic, 10);
    path_pub = nh.advertise<visualization_msgs::Marker>("/path", 10);

}

void NaivePlanner::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    odometry_information = *msg;
    odom_received = true;
}

void NaivePlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    goal = *msg;
    goal.pose.position.z = 1;
    goal_recieved = true;
}

bool NaivePlanner::go(geometry_msgs::Pose& target_)
{
    return false;
}

double NaivePlanner::goalDistance(geometry_msgs::Pose pose, geometry_msgs::Point goal){
    double x_diff = goal.x - pose.position.x, y_diff = goal.y - pose.position.y, z_diff = goal.z - pose.position.z;
    double euc = sqrt(x_diff*x_diff+y_diff*y_diff+z_diff*z_diff);
    return euc;
}


void NaivePlanner::run(void)
{
    ROS_INFO("Comienzo");
    initMarkerMsgs();
    bool plan_sent = false;
    ros::Rate rate(0.1);
    while(ros::ok() && !plan_sent){
        while(!odom_received)
            rate.sleep();
        bool success = false;
        if(goal_recieved){
            //Clear markers
            marker_msg.points.clear();
            marker_lines_msg.points.clear();  
            //Populate markers
            marker_msg.points.push_back(odometry_information.pose.position);
            marker_msg.points.push_back(goal.pose.position);
            marker_lines_msg.points.push_back(odometry_information.pose.position);
            marker_lines_msg.points.push_back(goal.pose.position);
            geometry_msgs::PoseArray naive_plan;
            naive_plan.header.frame_id = odometry_information.header.frame_id;
            naive_plan.header.stamp = odometry_information.header.stamp;
            naive_plan.poses.push_back(odometry_information.pose);
            naive_plan.poses.push_back(goal.pose);
            //Publish msgs
            path_pub.publish(marker_msg);
            path_pub.publish(marker_lines_msg);
            waypoints_pub.publish(naive_plan);
            plan_sent = true;
        }
        rate.sleep();
    }

    
}


void NaivePlanner::initMarkerMsgs(){
    //Configure markers
    marker_msg.action = marker_msg.MODIFY;
    marker_msg.header.frame_id = "odom";
    marker_msg.color.a = 0.7;
    marker_msg.color.g = 1;
    marker_msg.color.b = 1;
    marker_msg.color.r = 1;
    marker_msg.scale.x = 0.1;
    marker_msg.scale.y = 0.1;
    marker_msg.scale.z = 0.1;
    marker_msg.pose.orientation.x = 0;
    marker_msg.pose.orientation.y = 0;
    marker_msg.pose.orientation.z = 0;
    marker_msg.pose.orientation.w = 1;
    marker_msg.type = marker_lines_msg.SPHERE_LIST;
    marker_msg.id = 0;


    marker_lines_msg.action = marker_lines_msg.MODIFY;
    marker_lines_msg.header.frame_id = "odom";
    marker_lines_msg.color.a = 0.7;
    marker_lines_msg.color.g = 1;
    marker_lines_msg.color.b = 1;
    marker_lines_msg.color.r = 1;
    marker_lines_msg.scale.x = 0.05;
    marker_lines_msg.scale.y = 0.05;
    marker_lines_msg.scale.z = 0.05;
    marker_lines_msg.pose.orientation.x = 0;
    marker_lines_msg.pose.orientation.y = 0;
    marker_lines_msg.pose.orientation.z = 0;
    marker_lines_msg.pose.orientation.w = 1;
    marker_lines_msg.type = marker_lines_msg.LINE_LIST;
    marker_lines_msg.id = 1;
}
