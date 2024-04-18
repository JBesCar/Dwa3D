#include <global_planning.h>

/*
    Function used to check if a pose is valid in the octomap

    WARNING!! Only checks if the pose is occupied!!

    Returs true if position free or unknown
*/
bool octomapStateValidityChecker(const ompl::base::State *state, octomap::OcTree* octomap)
{   
    bool isValid = false;
    // OMPL state to (x, y, z)
    const auto queryPosition = state->as<ompl::base::RealVectorStateSpace::StateType>();
    double x = queryPosition->values[0];
    double y = queryPosition->values[1];
    double z = queryPosition->values[2];
    // Search the position in th octomap
    auto node = octomap->search(x,y,z);
    // If the position is unknown == NULL
    if(node == NULL){
        isValid = true;
    }else if(octomap->isNodeOccupied(node)){
        isValid = false;
    }else{
        isValid = true;
    }
    return isValid;
}
/*
    Function used to balance the different objectives involded in the global planning
*/
ompl::base::OptimizationObjectivePtr getBalancedObjective(const ompl::base::SpaceInformationPtr& si, 
                                                        ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal)
{
    ompl::base::OptimizationObjectivePtr headingObj(new HeadingObjective(si, goal));
    ompl::base::OptimizationObjectivePtr lengthObj(new ompl::base::PathLengthOptimizationObjective(si));
    return 0.5 * lengthObj +  0.5 * headingObj; 
}


/*
    Class that is in charge of performing the global planning with OMPL
*/
GlobalPlanner::GlobalPlanner(ros::NodeHandle& nh)
{
    //Set bools to faĺse
    odom_received = false;
    goal_recieved = false;
    octomap_recieved = false;

    //Load params
    nh.getParam("/XMIN", XMIN);
    nh.getParam("/XMAX", XMAX);
    nh.getParam("/YMIN", YMIN);
    nh.getParam("/YMAX", YMAX);
    nh.getParam("/ZMIN", ZMIN);
    nh.getParam("/ZMAX", ZMAX);
    nh.param("/max_planning_time", max_planning_time, 1.0);
    nh.param("/odom_topic", odom_topic, default_odom_topic);
    nh.param("/goal_topic", goal_topic, default_goal_topic);
    nh.param("/octomap_topic", octomap_topic, default_octomap_topic);
    nh.param("/markers_path_topic", markers_path_topic, default_markers_path_topic);
    nh.param("/waypoints_topic", waypoints_topic, default_waypoints_topic);

    //Set publishers and subscribers
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(odom_topic,10,&GlobalPlanner::poseCallback,this);
    goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(goal_topic,10,&GlobalPlanner::goalCallback,this);
    octomap_sub = nh.subscribe<octomap_msgs::Octomap>(octomap_topic, 10, &GlobalPlanner::octomapCallback, this);
    path_pub = nh.advertise<visualization_msgs::Marker>(markers_path_topic, 10);
    waypoints_pub = nh.advertise<geometry_msgs::PoseArray>(waypoints_topic, 10);
}

void GlobalPlanner::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
    odom_received = true;
}

void GlobalPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    goal = msg->pose;
    goal.position.z = 0.8;
    goal_recieved = true;
}

void GlobalPlanner::octomapCallback(octomap_msgs::Octomap msg){
    last_octomap_msg = msg;
    octomap_recieved = true;
}

octomap::OcTree *GlobalPlanner::getOctomap()
{
    return (octomap::OcTree *)octomap_msgs::msgToMap(last_octomap_msg);
}


double GlobalPlanner::goalDistance(geometry_msgs::Pose pose, geometry_msgs::Point goal){
    double x_diff = goal.x - pose.position.x, y_diff = goal.y - pose.position.y, z_diff = goal.z - pose.position.z;
    double euc = sqrt(x_diff*x_diff+y_diff*y_diff+z_diff*z_diff);
    return euc;
}


void GlobalPlanner::run(void)
{
    ROS_INFO("Global Planer Started");
    initMarkerMsgs();
    bool plan_sent = false;
    ros::Rate rate(0.1);
    //Send plan only once
    while(ros::ok() && !plan_sent){
        //Wait until current_pose is recieved
        while(!odom_received)
            rate.sleep();
        bool success = false;
        // Try to plan if both goal and octomap have been recieved
        if(goal_recieved && octomap_recieved){
            //Process the last Octomap msg       
            octomap = getOctomap();
            //Plan     
            std::vector<geometry_msgs::Point> path = plan(current_pose.pose, goal);
            //Prepare the PoseArray msg and the visual markers to show in rviz
            marker_msg.header.stamp = ros::Time::now();
            marker_lines_msg.header.stamp = ros::Time::now();
            if(path.size() > 0){
                geometry_msgs::PoseArray waypoints_list;
                waypoints_list.header.frame_id = current_pose.header.frame_id;
                marker_msg.points.clear();
                marker_lines_msg.points.clear();
                for(int i = 0; i < path.size(); i++){
                    //Populate marker
                    auto point = path[i];
                    marker_msg.points.push_back(point);
                    //Populate waypoint
                    geometry_msgs::Pose waypoint;
                    waypoint.position.x = point.x;
                    waypoint.position.y = point.y;
                    waypoint.position.z = point.z;
                    waypoints_list.poses.push_back(waypoint);
                    //Trace line for visualization
                    if(i < path.size() - 1){
                        marker_lines_msg.points.push_back(point);
                        marker_lines_msg.points.push_back(path[i+1]);
                    }
                }
                //Publish msgs
                path_pub.publish(marker_msg);
                path_pub.publish(marker_lines_msg);
                waypoints_pub.publish(waypoints_list);
                plan_sent = true;
            }
        }else if(!octomap_recieved){
            ROS_WARN("Waiting to recieve octomap");
        }else{
            ROS_WARN("Waiting to recieve goal");
        }
        rate.sleep();
    }
}

std::vector<geometry_msgs::Point> GlobalPlanner::plan(geometry_msgs::Pose start_pose,
                                                    geometry_msgs::Pose end_pose)
{
    //Vector of points that will return the solution
    std::vector<geometry_msgs::Point> points; 

    //Construct the state space we are planning in: R^3
    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(3));

    //Set the bounds for the R^3 space
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(-5);
    bounds.setHigh(5);
    space->setBounds(bounds);

    //Construct an instance of  space information from this state space
    auto si(std::make_shared<ompl::base::SpaceInformation>(space));

    //Set state validity checking for this space
    si->setStateValidityChecker(boost::bind(octomapStateValidityChecker, _1, octomap));

    //Set the motion validity checking for this space
    si->setMotionValidator(ompl::base::MotionValidatorPtr(new octomapMotionValidator(si, octomap)));

    //Create start state
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(space);
    start[0] = start_pose.position.x;
    start[1] = start_pose.position.y;
    start[2] = 1.0;//start_pose.position.z;

    //Create goal state
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(space);
    goal[0] = end_pose.position.x;
    goal[1] = end_pose.position.y;
    goal[2] = 1.0;//end_pose.position.z;

    //Create a problem definition
    auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));

    //Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    //Set optimization objective
    //ompl::base::OptimizationObjectivePtr headingObj(new HeadingObjective(si, goal));
    pdef->setOptimizationObjective(getBalancedObjective(si, goal)); //headingObj

    //Create a planner for the defined space
    auto planner(std::make_shared<ompl::geometric::RRTstar>(si));

    //Set the problem definition we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    //Setup the planner
    planner->setup();

    //Print the settings for this space
    si->printSettings(std::cout);

    //Print the problem definition settings
    pdef->print(std::cout);

    //Attempt to solve the problem within the given planning time
    ompl::base::PlannerStatus solved = planner->ompl::base::Planner::solve(max_planning_time);
    if (solved)
    {
        // Get the solution path
        ompl::base::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;
        //Print the path to screen
        path->print(std::cout);
        //Transform the OMPL plan to a geometry_msgs::Point vector
        ompl::geometric::PathGeometric pathGeometric(*path->as<ompl::geometric::PathGeometric>());
        for(auto state : pathGeometric.getStates()){
            geometry_msgs::Point point;
            point.x = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
            point.y = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
            point.z = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];
            points.push_back(point);
        }
    }
    else{
        std::cout << "No solution found" << std::endl;
    }
        
    return points;
}



void GlobalPlanner::initMarkerMsgs(){
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
