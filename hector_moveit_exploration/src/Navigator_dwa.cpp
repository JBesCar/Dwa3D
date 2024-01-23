#include <Navigator_dwa.h>

Navigator_dwa::Navigator_dwa(ros::NodeHandle& nh) : trajectory_client("/action/trajectory",true)
{
    odom_received = false;
    trajectory_received = false;
    goal_recieved = false;
    collision = false;
    nh.getParam("/XMIN", XMIN);
    nh.getParam("/XMAX", XMAX);
    nh.getParam("/YMIN", YMIN);
    nh.getParam("/YMAX", YMAX);
    nh.getParam("/ZMIN", ZMIN);
    nh.getParam("/ZMAX", ZMAX);
    nh.param("/planner", globalPanner, defaultPlanner);
    nh.param("/odom_topic", odom_topic, default_odom_topic);
    nh.param("/goal_topic", goal_topic, default_goal_topic);

    base_sub = nh.subscribe<geometry_msgs::PoseStamped>(odom_topic,10,&Navigator_dwa::poseCallback,this);
    goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(goal_topic,10,&Navigator_dwa::goalCallback,this);
    plan_sub = nh.subscribe<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",1,&Navigator_dwa::planCallback,this);

    move_group.reset(new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP));
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    kmodel = robot_model_loader.getModel();
    planning_scene_service = nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
    move_group->setPlannerId(globalPanner);
    move_group->setNumPlanningAttempts(10);
    move_group->setWorkspace(XMIN,YMIN,ZMIN,XMAX,YMAX,ZMAX);
    start_state.reset(new robot_state::RobotState(move_group->getRobotModel()));
    planning_scene.reset(new planning_scene::PlanningScene(kmodel));   
}

void Navigator_dwa::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    odometry_information = msg->pose;
    odom_received = true;
}

void Navigator_dwa::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    goal = msg->pose;
    goal.position.z = 1;
    goal_recieved = true;
}

void Navigator_dwa::planCallback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg)
{
    if(!odom_received) return;
    trajectory.clear();
    for(auto robot_traj: msg->trajectory){
        for(auto point : robot_traj.multi_dof_joint_trajectory.points){
            geometry_msgs::Pose waypoint;
            waypoint.position.x = point.transforms[0].translation.x;
            waypoint.position.y = point.transforms[0].translation.y;
            waypoint.position.z = point.transforms[0].translation.z;

            waypoint.orientation.x = point.transforms[0].rotation.x;
            waypoint.orientation.y = point.transforms[0].rotation.y;
            waypoint.orientation.z = point.transforms[0].rotation.z;
            waypoint.orientation.w = point.transforms[0].rotation.w;

            trajectory.push_back(waypoint);
        }
    }
    trajectory_received = true;
}

void Navigator_dwa::collisionCallback(const hector_moveit_actions::ExecuteDroneTrajectoryFeedbackConstPtr& feedback)
{
    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components = moveit_msgs::PlanningSceneComponents::OCTOMAP;
    if(planning_scene_service.call(srv)){
        this->planning_scene->setPlanningSceneDiffMsg(srv.response.scene);
        octomap_msgs::Octomap octomap = srv.response.scene.world.octomap.octomap;
        octomap::OcTree* current_map = (octomap::OcTree*)octomap_msgs::msgToMap(octomap);

/*         
        double resolution = current_map->getResolution();
        int unknown = 0, known = 0;
        for(double ix=XMIN;ix<XMAX;ix+=resolution){
            for(double iy=YMIN;iy<YMAX;iy+=resolution){
                for(double iz=ZMIN;iz<ZMAX;iz+=resolution){
                    if(!current_map->search(ix,iy,iz))
                        unknown++;
                    else
                        known++;
                }
            }
        }
        double rate = known*100.0/(float)(unknown+known);
        std_msgs::Float64 msg;
        msg.data = rate;
        */

        //delete current_map;
        std::vector<size_t> invalid_indices;
        this->isPathValid = this->planning_scene->isPathValid(plan_start_state,plan_trajectory,PLANNING_GROUP,true,&invalid_indices);
        ros::spinOnce();
        bool too_close = false;
        for(int i=0;i<invalid_indices.size();i++){
            for(int j=0;j<plan_trajectory.multi_dof_joint_trajectory.points[invalid_indices[i]].transforms.size();j++){
                
                double x = plan_trajectory.multi_dof_joint_trajectory.points[invalid_indices[i]].transforms[j].translation.x;
                double y = plan_trajectory.multi_dof_joint_trajectory.points[invalid_indices[i]].transforms[j].translation.y;
                double z = plan_trajectory.multi_dof_joint_trajectory.points[invalid_indices[i]].transforms[j].translation.z;

                double dist = sqrt(pow(x-odometry_information.position.x,2) + pow(y-odometry_information.position.y,2) + pow(z-odometry_information.position.z,2));
                if(dist < 0.5) too_close = true;
            }
        }
/*         
        if(!isPathValid && !too_close){
            //TODO: this->move_group->stop(); When migrating to complete MoveIt! ExecuteService, this will work as expected.
            this->trajectory_client.cancelAllGoals();
            this->collision = true;
            ROS_INFO("Trajectory is now in collision with the world");
        } 
         */
         //Check if the predicted pose is valid
        std::vector<double> predicted_state_(7);
        predicted_state_[0] = feedback->current_pose.position.x;
        predicted_state_[1] = feedback->current_pose.position.y;
        predicted_state_[2] = feedback->current_pose.position.z;
        predicted_state_[3] = feedback->current_pose.orientation.x;
        predicted_state_[4] = feedback->current_pose.orientation.y;
        predicted_state_[5] = feedback->current_pose.orientation.z;
        predicted_state_[6] = feedback->current_pose.orientation.w;
        robot_state::RobotState predicted_state(move_group->getRobotModel());
        predicted_state.setVariablePositions(predicted_state_);
        if(this->planning_scene->isStateColliding(predicted_state,PLANNING_GROUP,false)){
            this->trajectory_client.cancelAllGoals();
            ROS_INFO("Predicted pose is DANGEROUS");
            //enterRecoveryMode(10, 0.4);
        } 

    }
    else
        ROS_INFO("Couldn't fetch the planning scene");
}

bool Navigator_dwa::go(geometry_msgs::Pose& target_)
{
    std::vector<double> target(7);
    target[0] = target_.position.x;
    target[1] = target_.position.y;
    target[2] = target_.position.z;
    target[3] = target_.orientation.x;
    target[4] = target_.orientation.y;
    target[5] = target_.orientation.z;
    target[6] = target_.orientation.w;
    
    std::vector<double> start_state_(7);
    start_state_[0] = odometry_information.position.x;
    start_state_[1] = odometry_information.position.y;
    start_state_[2] = odometry_information.position.z;
    start_state_[3] = odometry_information.orientation.x;
    start_state_[4] = odometry_information.orientation.y;
    start_state_[5] = odometry_information.orientation.z;
    start_state_[6] = odometry_information.orientation.w;
    ros::WallTime start_, end_;
    this->move_group->setJointValueTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    this->collision = false;
    ROS_INFO("Try to start from [%lf,%lf,%lf]",odometry_information.position.x,odometry_information.position.y,odometry_information.position.z);
    ROS_INFO("Try to go to [%lf,%lf,%lf]",target_.position.x,target_.position.y,target_.position.z);
    this->start_state->setVariablePositions(start_state_);
    //We must check if the start state is safe, otherwise the drone could be in a trap situation
    if(this->planning_scene->isStateColliding(*this->start_state,PLANNING_GROUP,false)){
        ROS_INFO("TRAP SITUATION, SEARCHING FOR A SAFE INITIAL POSITION");
        robot_state::RobotState safe_state = searchSafePositionAround(20, 1);
        this->start_state.reset(new robot_state::RobotState(safe_state));
    }
    this->move_group->setStartState(*this->start_state);
    ROS_INFO("Send for planning");
    start_ = ros::WallTime::now();
    moveit::planning_interface::MoveItErrorCode plan_error_code = move_group->plan(plan);
    this->isPathValid = plan_error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    if(this->isPathValid){
        
        this->plan_start_state = plan.start_state_;
        this->plan_trajectory = plan.trajectory_;
        while(!trajectory_received){
            ROS_INFO("Waiting for trajectory");
            ros::Duration(0.2).sleep();
        }
        hector_moveit_actions::ExecuteDroneTrajectoryGoal goal;
        for(int i=0;i<trajectory.size();i++){
            if(i==0){
                double y_diff = trajectory[i].position.y - odometry_information.position.y;
                double x_diff = trajectory[i].position.x - odometry_information.position.x;
                double yaw = atan2(y_diff,x_diff);
                if(fabs(y_diff)>EPSILON || fabs(x_diff)>EPSILON ){ //Prevent 0 division
                    tf::Quaternion q = tf::createQuaternionFromYaw(yaw+M_PI);
                    trajectory[i].orientation.x = q.x();
                    trajectory[i].orientation.y = q.y();
                    trajectory[i].orientation.z = q.z();
                    trajectory[i].orientation.w = q.w();
                }
            }
            else if(i+1<trajectory.size()){
                geometry_msgs::Pose next_waypoint = trajectory[i+1];
                double y_diff = next_waypoint.position.y - trajectory[i].position.y;
                double x_diff = next_waypoint.position.x - trajectory[i].position.x;
                double yaw = atan2(y_diff,x_diff);
                
                if(fabs(y_diff)>EPSILON || fabs(x_diff)>EPSILON ){ //Prevent 0 division
                    
                    tf::Quaternion q = tf::createQuaternionFromYaw(yaw+M_PI);
                    trajectory[i+1].orientation.x = q.x();
                    trajectory[i+1].orientation.y = q.y();
                    trajectory[i+1].orientation.z = q.z();
                    trajectory[i+1].orientation.w = q.w();
                }
            }
            
            //if ((i%10 == 0 && i < trajectory.size()-10) || i == (trajectory.size()-1)) {                
                goal.trajectory.push_back(trajectory[i]);
            //}
            
        }    
        end_ = ros::WallTime::now();
        ROS_INFO("Planning time(ms): %f", (end_ - start_).toNSec() * 1e-6);
        ROS_INFO("Trajectory size: %d", goal.trajectory.size());
        trajectory_client.sendGoal(goal,actionlib::SimpleActionClient<hector_moveit_actions::ExecuteDroneTrajectoryAction>::SimpleDoneCallback(),
                            actionlib::SimpleActionClient<hector_moveit_actions::ExecuteDroneTrajectoryAction>::SimpleActiveCallback(),
                            boost::bind(&Navigator_dwa::collisionCallback,this,_1));
        trajectory_client.waitForResult(ros::Duration(0));
        this->trajectory_received = false;
        this->odom_received = false;
    }
    return this->isPathValid;
}

double Navigator_dwa::goalDistance(geometry_msgs::Pose pose, geometry_msgs::Point goal){
    double x_diff = goal.x - pose.position.x, y_diff = goal.y - pose.position.y, z_diff = goal.z - pose.position.z;
    double euc = sqrt(x_diff*x_diff+y_diff*y_diff+z_diff*z_diff);
    return euc;
}


robot_state::RobotState Navigator_dwa::searchSafePositionAround(int n_searchs_recovery, double search_dist){
    double max_dist = 0;
    std::vector<double> current_values(7);
    current_values[0] = odometry_information.position.x;
    current_values[1] = odometry_information.position.y;
    current_values[2] = odometry_information.position.z;
    current_values[3] = odometry_information.orientation.x;
    current_values[4] = odometry_information.orientation.y;
    current_values[5] = odometry_information.orientation.z;
    current_values[6] = odometry_information.orientation.w;
    robot_state::RobotState current_state(this->kmodel);
    current_state.setVariablePositions(current_values);
    robot_state::RobotState safe_state(current_state);
    robot_state::RobotState random_state(current_state);
    const robot_state::JointModelGroup *joint_model_group(current_state.getJointModelGroup(PLANNING_GROUP));
    //Search for a safe place around the current state
    for(int i = 0; i < n_searchs_recovery; i++){
        //Select a random position in the search area
        random_state.setToRandomPositionsNearBy(joint_model_group,current_state,search_dist);
        //Get the distance to the closest collision
        double dist = this->planning_scene->distanceToCollision(random_state);
        //Keep the safest
        if(dist > max_dist){
            max_dist  = dist;
            safe_state = robot_state::RobotState(random_state);
        }
    }
    return safe_state;
}


void Navigator_dwa::enterRecoveryMode(int n_searchs_recovery, double search_dist){
    double max_dist = 0;
    std::vector<double> current_values(7);
    current_values[0] = odometry_information.position.x;
    current_values[1] = odometry_information.position.y;
    current_values[2] = odometry_information.position.z;
    current_values[3] = odometry_information.orientation.x;
    current_values[4] = odometry_information.orientation.y;
    current_values[5] = odometry_information.orientation.z;
    current_values[6] = odometry_information.orientation.w;
    robot_state::RobotState current_state(this->kmodel);
    current_state.setVariablePositions(current_values);
    robot_state::RobotState safe_state(current_state);
    robot_state::RobotState random_state(current_state);
    const robot_state::JointModelGroup *joint_model_group(current_state.getJointModelGroup(PLANNING_GROUP));
    //Search for a safe place around the current state
    for(int i = 0; i < n_searchs_recovery; i++){
        //Select a random position in the search area
        random_state.setToRandomPositionsNearBy(joint_model_group,current_state,search_dist);
        //Get the distance to the closest collision
        double dist = this->planning_scene->distanceToCollision(random_state);
        //Keep the safest
        if(dist > max_dist){
            max_dist  = dist;
            safe_state = robot_state::RobotState(random_state);
        }
    }

    // Configure path to recovery pose
    hector_moveit_actions::ExecuteDroneTrajectoryGoal goal;
    const double *safe_values = safe_state.getVariablePositions();
    trajectory.clear();
    geometry_msgs::Pose current_point, safe_point;
    current_point.position.x = current_values[0];
    current_point.position.y = current_values[1];
    current_point.position.z = current_values[2];
    current_point.orientation.x = 0;
    current_point.orientation.y = 0;
    current_point.orientation.z = 0;
    current_point.orientation.w = 0;

    safe_point.position.x = safe_values[0];
    safe_point.position.y = safe_values[1];
    safe_point.position.z = safe_values[2];
    safe_point.orientation.x = 0;
    safe_point.orientation.y = 0;
    safe_point.orientation.z = 0;
    safe_point.orientation.w = 0;

    goal.trajectory.push_back(current_point);
    goal.trajectory.push_back(safe_point);
    ROS_INFO("Entering Recovery Mode");
    std::cout << "Current Position: " << std::endl;
    std::cout << "X: " << current_values[0] << " Y: " << 
                current_values[1] << " Z: " <<current_values[2] << std::endl;
    std::cout << "Safe Position: " << std::endl;
    std::cout << "X: " << safe_values[0] << " Y: " << 
                safe_values[1] << " Z: " <<safe_values[2] << std::endl;
    trajectory_client.sendGoal(goal,actionlib::SimpleActionClient<hector_moveit_actions::ExecuteDroneTrajectoryAction>::SimpleDoneCallback(),
                            actionlib::SimpleActionClient<hector_moveit_actions::ExecuteDroneTrajectoryAction>::SimpleActiveCallback(),
                            actionlib::SimpleActionClient<hector_moveit_actions::ExecuteDroneTrajectoryAction>::SimpleFeedbackCallback());
}


void Navigator_dwa::run(void)
{
    ROS_INFO("Comienzo");
    ros::Rate rate(2);
    while(ros::ok()){
        while(!odom_received)
            rate.sleep();
        bool success = false;
        if(goal_recieved){
            do{
                goal.orientation.x = odometry_information.orientation.x;
                goal.orientation.y = odometry_information.orientation.y;
                goal.orientation.z = odometry_information.orientation.z;
                goal.orientation.w = odometry_information.orientation.w;
                success = go(goal);
                
                ros::spinOnce();
                rate.sleep();
            }while(!success);
            
/*             while(!trajectory_client.getState().isDone()){
                //trajectory_client.waitForResult(ros::Duration(rate));
                ros::spinOnce();
                rate.sleep();
            } */
            Result result = *trajectory_client.getResult();
            if (result.result_code == Result::SUCCESSFUL) { //goalDistance(odometry_information, goal.position) < 0.5
                ROS_INFO_STREAM("Aterrizaje finalizado");
                std::cout<<" COORDENADAS DEL OBJETIVO: X = "<<goal.position.x<<"m , Y = "<<goal.position.y<<"m"<<std::endl;
                std::cout<<" COORDENADAS ALCANZADAS: X = "<<odometry_information.position.x<<"m , Y = "<<odometry_information.position.y<<"m"<<std::endl;
                ros::shutdown();
            }
        }
    }
}
