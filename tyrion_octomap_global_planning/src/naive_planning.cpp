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
    ros::Rate rate(0.1);
    while(ros::ok()){
        while(!odom_received)
            rate.sleep();
        bool success = false;
        if(goal_recieved){            
            geometry_msgs::PoseArray naive_plan;
            naive_plan.header.frame_id = odometry_information.header.frame_id;
            naive_plan.header.stamp = odometry_information.header.stamp;
            naive_plan.poses.push_back(odometry_information.pose);
            naive_plan.poses.push_back(goal.pose);
            waypoints_pub.publish(naive_plan);
        }
        rate.sleep();
    }
}
