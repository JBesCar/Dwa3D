#include <global_planning.h>


bool octomapStateValidityChecker(const ompl::base::State *state, octomap::OcTree* octomap)
{   
    bool isValid = false;
    const auto queryPosition = state->as<ompl::base::RealVectorStateSpace::StateType>();
    double x = queryPosition->values[0];
    double y = queryPosition->values[1];
    double z = queryPosition->values[2];
    auto node = octomap->search(x,y,z);
    if(node == NULL){
        isValid = true;
    }else if(octomap->isNodeOccupied(node)){
        isValid = false;
    }else{
        isValid = true;
    }
    return isValid;
}

ompl::base::OptimizationObjectivePtr getBalancedObjective(const ompl::base::SpaceInformationPtr& si, 
                                                        ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal)
{
    ompl::base::OptimizationObjectivePtr headingObj(new HeadingObjective(si, goal));
    ompl::base::OptimizationObjectivePtr lengthObj(new ompl::base::PathLengthOptimizationObjective(si));
    return 0.5 * lengthObj +  2 * headingObj; 
}

GlobalPlanner::GlobalPlanner(ros::NodeHandle& nh)
{
    odom_received = false;
    trajectory_received = false;
    goal_recieved = false;
    collision = false;
    octomap_recieved = false;
    nh.getParam("/XMIN", XMIN);
    nh.getParam("/XMAX", XMAX);
    nh.getParam("/YMIN", YMIN);
    nh.getParam("/YMAX", YMAX);
    nh.getParam("/ZMIN", ZMIN);
    nh.getParam("/ZMAX", ZMAX);
    nh.param("/odom_topic", odom_topic, default_odom_topic);
    nh.param("/goal_topic", goal_topic, default_goal_topic);
    nh.param("/planner_service", planner_service, std::string("/voxblox_rrt_planner/plan"));
    nh.param("/publish_plath_service", publish_plath_service,std::string("/voxblox_rrt_planner/publish_path"));

    base_sub = nh.subscribe<geometry_msgs::PoseStamped>(odom_topic,10,&GlobalPlanner::poseCallback,this);
    goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(goal_topic,10,&GlobalPlanner::goalCallback,this);
    plan_sub = nh.subscribe<visualization_msgs::MarkerArray>("/voxblox_rrt_planner/path",10,&GlobalPlanner::planCallback,this);
    octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, &GlobalPlanner::octomapCallback, this);
    path_pub = nh.advertise<visualization_msgs::Marker>("/path", 10);
    waypoints_pub = nh.advertise<geometry_msgs::PoseArray>("/waypoint_list", 10);



}

void GlobalPlanner::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    odometry_information = *msg;
    odom_received = true;
    //request.request.start_pose = *msg;
    //request.request.start_pose.pose.position.z = 1;
}

void GlobalPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    goal = msg->pose;
    goal.position.z = 1;
    goal_recieved = true;
    //request.request.goal_pose = *msg;
    //request.request.goal_pose.pose.position.z = 1;
}

void GlobalPlanner::planCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    std_srvs::Empty path_request;
    try {
        ROS_DEBUG_STREAM("Service name: " << publish_plath_service);
        if (!ros::service::call(publish_plath_service, path_request)) {
            ROS_WARN_STREAM("Couldn't call service: " << publish_plath_service);
        }
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Service Exception: " << e.what());
    }
}

void GlobalPlanner::octomapCallback(octomap_msgs::Octomap msg){
    last_octomap_msg = msg;
    octomap_recieved = true;
}

octomap::OcTree *GlobalPlanner::getOctomap()
{
    return (octomap::OcTree *)octomap_msgs::msgToMap(last_octomap_msg);
}

bool GlobalPlanner::go(geometry_msgs::Pose& target_)
{
    return false;
}

double GlobalPlanner::goalDistance(geometry_msgs::Pose pose, geometry_msgs::Point goal){
    double x_diff = goal.x - pose.position.x, y_diff = goal.y - pose.position.y, z_diff = goal.z - pose.position.z;
    double euc = sqrt(x_diff*x_diff+y_diff*y_diff+z_diff*z_diff);
    return euc;
}


void GlobalPlanner::run(void)
{
    ROS_INFO("Comienzo");
    initMarkerMsgs();
    bool plan_sent = false;
    ros::Rate rate(0.1);
    while(ros::ok() && !plan_sent){
        while(!odom_received)
            rate.sleep();
        bool success = false;
        if(goal_recieved && octomap_recieved){       
            octomap = getOctomap();     
            std::vector<geometry_msgs::Point> path = plan(odometry_information.pose, goal);
            
            marker_msg.header.stamp = ros::Time::now();
            marker_lines_msg.header.stamp = ros::Time::now();
            if(path.size() > 0){
                geometry_msgs::PoseArray waypoints_list;
                waypoints_list.header.frame_id = odometry_information.header.frame_id;
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
    std::vector<geometry_msgs::Point> points; 
    // construct the state space we are planning in
    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(3));

    // set the bounds for the R^3 part of SE(3)
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(-5);
    bounds.setHigh(5);

    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ompl::base::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker(boost::bind(octomapStateValidityChecker, _1, octomap));

    //set the motion validity checking for this space
    si->setMotionValidator(ompl::base::MotionValidatorPtr(new octomapMotionValidator(si, octomap)));



    // create start state
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(space);
    start[0] = start_pose.position.x;
    start[1] = start_pose.position.y;
    start[2] = 1.0;//start_pose.position.z;

    // create goal state
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(space);
    goal[0] = end_pose.position.x;
    goal[1] = end_pose.position.y;
    goal[2] = 1.0;//end_pose.position.z;

    // create a prompl::baselem instance
    auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    //set optimization objective
    //ompl::base::OptimizationObjectivePtr headingObj(new HeadingObjective(si, goal));
    pdef->setOptimizationObjective(getBalancedObjective(si, goal)); //headingObj

    // create a planner for the defined space
    auto planner(std::make_shared<ompl::geometric::RRTstar>(si));

    // set the prompl::baselem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();

    // print the settings for this space
    si->printSettings(std::cout);

    // print the prompl::baselem settings
    pdef->print(std::cout);

    // attempt to solve the prompl::baselem within one second of planning time
    ompl::base::PlannerStatus solved = planner->ompl::base::Planner::solve(1.0);

    if (solved)
    {
        // get the goal representation from the prompl::baselem definition (not the same as the goal state)
        // and inquire about the found path
        ompl::base::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        path->print(std::cout);
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
    //Configure marker
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
