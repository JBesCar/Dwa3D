
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/State.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


bool isStateValid(const ompl::base::State *state)
{
    // cast the abstract state type to the type we expect
    const auto *R3state = state->as<ompl::base::RealVectorStateSpace::StateType>();

    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return true;
}

std::vector<geometry_msgs::Point> plan()
{
    std::vector<geometry_msgs::Point> points; 
    // construct the state space we are planning in
    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(3));

    // set the bounds for the R^3 part of SE(3)
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);

    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ompl::base::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker(isStateValid);

    // create a random start state
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(space);
    start[0] = 0;
    start[1] = 0;
    start[2] = 0;
    //start.random();

    // create a random goal state
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(space);
    goal.random();

    // create a prompl::baselem instance
    auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
    auto planner(std::make_shared<ompl::geometric::RRTConnect>(si));

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

void planWithSimpleSetup()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(3));

    // set the bounds for the R^3 part of SE(3)
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);

    space->setBounds(bounds);

    // define a simple setup class
    ompl::geometric::SimpleSetup ss(space);

    // set state validity checking for this space
    ss.setStateValidityChecker([](const ompl::base::State *state)
                               { return isStateValid(state); });

    // create a random start state
    ompl::base::ScopedState<> start(space);
    start.random();

    // create a random goal state
    ompl::base::ScopedState<> goal(space);
    goal.random();

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // this call is optional, but we put it in to get more output information
    ss.setup();
    ss.print();

    // attempt to solve the prompl::baselem within one second of planning time
    ompl::base::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int argc, char ** argv)
{   
    ros::init(argc, argv, "ompl_example");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");
    ros::Publisher path_pub = node_handle.advertise<visualization_msgs::Marker>("/path", 10);
    //Configure marker
    visualization_msgs::Marker marker_msg, marker_lines_msg;
    marker_msg.action = marker_msg.ADD;
    marker_msg.header.frame_id = "odom";
    marker_msg.color.a = 1;
    marker_msg.color.g = 1;
    marker_msg.color.b = 0;
    marker_msg.color.r = 0;
    marker_msg.scale.x = 0.1;
    marker_msg.scale.y = 0.1;
    marker_msg.scale.z = 0.1;
    marker_msg.pose.orientation.x = 0;
    marker_msg.pose.orientation.y = 0;
    marker_msg.pose.orientation.z = 0;
    marker_msg.pose.orientation.w = 1;
    marker_msg.type = marker_lines_msg.SPHERE_LIST;
    marker_msg.id = 0;


    marker_lines_msg.action = marker_lines_msg.ADD;
    marker_lines_msg.header.frame_id = "odom";
    marker_lines_msg.color.a = 1;
    marker_lines_msg.color.g = 1;
    marker_lines_msg.color.b = 0;
    marker_lines_msg.color.r = 0;
    marker_lines_msg.scale.x = 0.05;
    marker_lines_msg.scale.y = 0.05;
    marker_lines_msg.scale.z = 0.05;
    marker_lines_msg.pose.orientation.x = 0;
    marker_lines_msg.pose.orientation.y = 0;
    marker_lines_msg.pose.orientation.z = 0;
    marker_lines_msg.pose.orientation.w = 1;
    marker_lines_msg.type = marker_lines_msg.LINE_LIST;
    marker_lines_msg.id = 1;
    

    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    std::vector<geometry_msgs::Point> path = plan();
    
    marker_msg.header.stamp = ros::Time::now();
    if(path.size() > 0){
        for(int i = 0; i < path.size(); i++){
            auto point = path[i];
            marker_msg.points.push_back(point);
            if(i < path.size() - 1){
                marker_lines_msg.points.push_back(point);
                marker_lines_msg.points.push_back(path[i+1]);
            }
        }
    }

    while(ros::ok()){
        path_pub.publish(marker_msg);
        path_pub.publish(marker_lines_msg);
    }
    
    std::cout << std::endl
              << std::endl;

    //planWithSimpleSetup();

    return 0;
}
