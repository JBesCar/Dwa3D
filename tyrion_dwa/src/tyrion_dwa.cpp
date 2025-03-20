#include <tyrion_dwa.h>

Dwa3d::Dwa3d(const ros::NodeHandle &nh,
             const ros::NodeHandle &nh_private,
             const double _R_drone, const double _T,  const double _delta_t,
             const double _vx_step, const double _vz_step, 
             const double _w_step, const double _aLin, 
             const double _aAng) : tf_listener_(tf_buffer_),
                                  nh_(nh), nh_private_(nh_private),
                                  R_drone(_R_drone), T(_T), delta_t(_delta_t),
                                  vx_step(_vx_step), vz_step(_vz_step),
                                  w_step(_w_step), aLin(_aLin), aAng(_aAng),
                                  filas_tot(((2*aLin*T/vx_step) + 1)*((2*aAng*T/w_step) + 1)*((2*aLin*T/vz_step) + 1)),
                                  comp_eval(filas_tot), 
                                  comp_eval_norm(filas_tot), // Para cada posible velocidad de la ventana
                                  G(filas_tot)                                     // Puntuaciones                                
{
    // Load parameters from ros server
    // DWA
    nh_private_.param("ALFA", ALFA, 0.3);
    nh_private_.param("Ky", Ky, 0.5);
    nh_private_.param("Kz", Kz, 0.5);
    nh_private_.param("BETA", BETA, 0.6);
    nh_private_.param("GAMMA", GAMMA, 0.1);
    nh_private_.param("goal_step", step, 1.0);
    nh_private_.param("subgoal_step", subgoal_step, 0.5);
    nh_private_.param("iter_update", iter_update, 150);
    //nh_private_.param("iter_obs", iter_obs, 10);
    iter_obs = int(delta_t / T);
    // Topics and frames
    nh_private_.param("cmd_vel_control_topic", cmd_vel_control_topic, std::string("/cmd_vel_control"));
    nh_private_.param("ground_truth_topic", ground_truth_topic, std::string("/floam/pose"));
    nh_private_.param("plan_topic", plan_topic, std::string("/waypoint_list"));
    nh_private_.param("current_vel_topic", current_vel_topic, std::string("/mavros/local_position/velocity_body"));
    nh_private_.param("cmd_frame_id", cmd_frame, std::string("odom"));
    //Ray casting
    nh_private_.param("r_search", r_search, 2.0);
    nh_private_.param("psi_beam_max", psi_beam_max, PI/2);
    nh_private_.param("theta_beam_max", theta_beam_max, PI/2);
    nh_private_.param("delta_psi", delta_psi, 10 * PI/180);
    nh_private_.param("delta_theta", delta_theta, 10 * PI/180);
    nh_private_.param("lambda_psi", lambda_psi, 0.5);
    nh_private_.param("lambda_theta", lambda_theta, 0.75);
    nh_private_.param("treat_unknown_as_occupied",treat_unknown_as_occupied,false);
    //Velocity limits
    nh_private_.param("vx_min", vx_min, 0.0);
    nh_private_.param("vx_max", vx_max, 0.3);
    nh_private_.param("w_max", w_max, PI/4);
    nh_private_.param("vz_max", vz_max,0.3);

    ROS_INFO("DWA Params loaded");
    std::cout << "ALPHA: " << ALFA << std::endl;
    std::cout << "BETA: " << BETA << std::endl;
    std::cout << "GAMMA: " << GAMMA << std::endl;
    std::cout << "Kz: " << Kz << std::endl;
    std::cout << "Ky: " << Ky << std::endl;
    std::cout << "R search: " << r_search << std::endl;
    
    // Initialize servers, topics and messages
    empty.linear.x = 0;
    empty.linear.y = 0;
    empty.linear.z = 0;
    empty.angular.x = 0;
    empty.angular.y = 0;
    empty.angular.z = 0;

    Vs = { 0, vx_max, -w_max, w_max, -vz_max, vz_max };
    std::cout << "Vx_min: " << Vs[0] << std::endl;
    std::cout << "Vx_max: " << Vs[1] << std::endl;
    std::cout << "w_min: " << Vs[2] << std::endl;
    std::cout << "w_max: " << Vs[3] << std::endl;
    std::cout << "Vz_min: " << Vs[4] << std::endl;
    std::cout << "Vz_max: " << Vs[5] << std::endl;
    
    //Vel command control topic
    vel_pub = nh_.advertise<geometry_msgs::Twist>(cmd_vel_control_topic, 10);
    // Debug and visual info
    vel_visual_pub = nh_.advertise<geometry_msgs::TwistStamped>("/vel_visual", 10);
    markers_debug_pub = nh_.advertise<visualization_msgs::Marker>("/markers_debug", 10);
    predicted_pose_pub = nh_.advertise<visualization_msgs::Marker>("/predicted_pose", 10);
    discarded_poses_pub = nh_.advertise<visualization_msgs::Marker>("/discarded_poses_marker", 10);
    DWA_visual_pub = nh_.advertise<tyrion_dwa::DynamicWindowMsg>("/DWA_visual_msg", 10);
    comp_time_pub = nh_.advertise<std_msgs::Float32>("/dwa_computational_time", 10);

    // Pose subscriber
    pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>(ground_truth_topic, 10, &Dwa3d::poseCallback, this);
    // Global plan susbcriber
    plan_sub = nh_.subscribe<geometry_msgs::PoseArray>(plan_topic, 10, &Dwa3d::planCallback, this);
    // Current Vel subscriber
    current_vel_sub = nh_.subscribe<geometry_msgs::TwistStamped>(current_vel_topic, 10, &Dwa3d::currentVelCallback, this);
    // Octomap topic
    octomap_sub = nh_.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, &Dwa3d::octomapCallback, this);
    //Mavros info about the UAV and services
    state_sub = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &Dwa3d::state_cb, this);
    extended_state_sub = nh_.subscribe<mavros_msgs::ExtendedState>("mavros/extended_state", 10, &Dwa3d::extended_state_cb, this);
    arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    set_cmd_vel_frame = nh_.serviceClient<mavros_msgs::SetMavFrame>("mavros/setpoint_velocity/mav_frame");
    ROS_INFO("Servers and topics created");

    executing = false;
    
    //TO-DO: Think if keep this here or in idle...
    // wait for FCU connection
    ros::Rate wait_rate(1.0);
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        wait_rate.sleep();
        ROS_INFO("FCU not ready, waiting...");
    }

    // Check if the drone is armed and
    // in position mode before going offboard
    while (ros::ok() && !current_state.armed &&
           current_state.mode != "POSITION")
    {
        ROS_INFO("Arm manually and enable position mode before going offboard");
        ros::spinOnce();
        wait_rate.sleep();
    }
    // Try going offboard
    bool offboard = false;
    do
    {
        ROS_INFO("Trying to go offboard");
        offboard = this->tryOffboard();
    } while (ros::ok() && !offboard);
    // Stablish the cmd_vel frame
    mavros_msgs::SetMavFrame set_frame_msg;
    set_frame_msg.request.mav_frame = set_frame_msg.request.FRAME_BODY_NED;
    set_cmd_vel_frame.call(set_frame_msg);
}

void Dwa3d::state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void Dwa3d::extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr &msg)
{
    current_extended_state = *msg;
}

void Dwa3d::planCallback(const geometry_msgs::PoseArray::ConstPtr &path)
{
    executing = true;
    trajectory = path->poses;
    goal_i = 1;
}

void Dwa3d::followPlan()
{
    ros::WallTime time_start_, time_end_;
    geometry_msgs::Point subgoal;
    geometry_msgs::Twist cmd_vel;
    std::array<double, 6> Vd, Vsd;
    geometry_msgs::Pose pose_predicted;
    static std::array<double, 3> current_vel = {0, 0, 0};
    std::array<double, 3> selected_vel = {0, 0, 0};
    ros::Rate loop_rate(1 / T);
    static int iteraciones = 0;
    subgoal = trajectory[goal_i].position;
    // Search for nearest subgoal
    /*             while(goalDistance(current_pose, subgoal) < 0.5 && goal_i < trajectory.size()){
                    goal_i++;
                    subgoal = trajectory[goal_i].position;
                } */
    // ROS_INFO("DWA Goal: (%f, %f, %f)", subgoal.x, subgoal.y, subgoal.z);

    time_start_ = ros::WallTime::now();

    if ((iteraciones % iter_obs) == 0)
        lecturaObstaculos = true;
    
    //Check if velocity estimation has been already recieved
    if (current_vel_recieved)
    {
        current_vel_recieved = false;
        current_vel[0] = current_vel_msg.linear.x;
        current_vel[1] = current_vel_msg.angular.z;
        current_vel[2] = current_vel_msg.linear.z;
    }

    //Initialize Search Space
    int filas_eval = 0, filas_x = 0, filas_w = 0, filas_z = 0; // Rows evaluated in the window
    Vd = {std::max(0.0, current_vel[0]) - aLin * T, std::max(0.0, current_vel[0]) + aLin * T,
          current_vel[1] - aAng * T, current_vel[1] + aAng * T,
          current_vel[2] - aLin * T, current_vel[2] + aLin * T}; // Dynamic Window Space: Vd
    Vsd = {std::max(Vs[0], Vd[0]), std::min(Vs[1], Vd[1]),
           std::max(Vs[2], Vd[2]), std::min(Vs[3], Vd[3]),
           std::max(Vs[4], Vd[4]), std::min(Vs[5], Vd[5])}; // Vsd = Intersection between Vs and Vd

    // Initilization of Cost Function and its terms 
    for (int i = 0; i < filas_tot * COLS; i++)
        comp_eval[i / COLS][i % COLS] = 0;
    for (int i = 0; i < filas_tot; i++)
        G[i] = 0;

    // Retrieve the Octomap from the server
    if (lecturaObstaculos)
    {
        octomap = getOctomap();
        lecturaObstaculos = false;
    }

    //Init markers for visualization and clear its contents
    visualization_msgs::Marker discarded_poses_debug;
    visualization_msgs::Marker casted_rays_markers;
    //resetMarkersContents(); //TO-DO reset only contents, not markers info
    initMarkers(&discarded_poses_debug, &casted_rays_markers);

    // Recorrer el espacio de búsqueda Vsd
    for (double vx = Vsd[0]; vx <= Vsd[1]; vx += vx_step)
    {
        vx = round(vx / vx_step) * vx_step;
        for (double w = Vsd[2]; w <= Vsd[3]; w += w_step)
        { //(Vsd[3]-Vsd[3]*vx/(4*std::max(0.01,Vsd[1])))
            w = round(w / w_step) * w_step;
            for (double vz = Vsd[4]; vz <= Vsd[5]; vz += vz_step)
            {
                vz = round(vz / vz_step) * vz_step;
                //Predict pose after applying (vx, vz, wz) during delta_t
                pose_predicted = simubot(vx, w, vz, delta_t); // iter_obs*T
                //Distance to the closest obstacle (from the predicted pose)
                double min_dist = minDistOctomap(current_pose, pose_predicted, octomap, vx, vz);
                double v = sqrt(vx * vx + vz * vz); 
                double wabs = fabs(w);

                // Markers for debug
                {
                    geometry_msgs::Point point;
                    point.x = pose_predicted.position.x;
                    point.y = pose_predicted.position.y;
                    point.z = pose_predicted.position.z;
                    discarded_poses_debug.points.push_back(point);
                }

                // Compute the terms for Vr (those inside Vsd that does not lead to collision)
                if (v <= sqrt(2 * aLin * min_dist) || BETA == 0) // w never leads to collision
                {
                    //Head Yaw Term
                    comp_eval[filas_eval][0] = calcYawHeading(pose_predicted, subgoal);
                    //Head Z Term
                    double heading_z = calcZHeading(pose_predicted, subgoal); // In meters!!
                    comp_eval[filas_eval][1] = fabs(heading_z);
                    // Distance Term
                    comp_eval[filas_eval][2] = min_dist;

                    // Velocities for debug
                    comp_eval[filas_eval][4] = vx;
                    comp_eval[filas_eval][5] = w;
                    comp_eval[filas_eval][6] = vz;
                    // Distance to waypoint for debug
                    comp_eval[filas_eval][7] = goalDistance(pose_predicted, subgoal);

                    // Velocity Term
                    if ((Kz > Ky) ||
                        (comp_eval[filas_eval][0] > 0.5 && Ky > Kz))
                    {
                        comp_eval[filas_eval][3] = vx / vx_max;
                    }
                    else
                    {
                        comp_eval[filas_eval][3] = 0;
                    }
                    filas_eval++;
                }
                filas_z++;
            }
            filas_w++;
        }
        filas_x++;
    }

    // Populate a message for visual information, useful for debugging
    tyrion_dwa::DynamicWindowMsg DWA_visual_msg;
    initDwaVisualMsg(&DWA_visual_msg, Vsd, filas_eval, filas_tot);

    // DEBUG
    discarded_poses_debug.header.stamp = ros::Time::now();
    discarded_poses_pub.publish(discarded_poses_debug);
    casted_rays_markers.header.stamp = ros::Time::now();
    
    // If filas_eval == 0 there is no safe velocity to apply
    if (filas_eval != 0)
    {
        // Compute maximum values in evaluation matrix
        double yaw_head_max, z_head_max, dist_max, vel_max;
        yaw_head_max = comp_eval[0][0];
        z_head_max = comp_eval[0][1];
        dist_max = r_search;
        vel_max = comp_eval[0][3];
        for (int i = 0; i < filas_eval; i++)
        {
            if (comp_eval[i][0] > yaw_head_max)
                yaw_head_max = comp_eval[i][0];
            if (comp_eval[i][1] > z_head_max)
                z_head_max = comp_eval[i][1];
            if (comp_eval[i][3] > vel_max)
                vel_max = comp_eval[i][3];
        }

        // Avoid divsions by 0
        if (yaw_head_max == 0)
            yaw_head_max = 1;
        if (z_head_max == 0)
            z_head_max = 1;
        if (dist_max == 0)
            dist_max = 1;
        if (vel_max == 0)
            vel_max = 1;

        // Optimize the cost function
        // TO-DO: Substitute brute force by some optimization library
        double max_G = G[0];
        int ind = 0;
        // Normalization and G computation
        for (int i = 0; i < filas_eval; i++)
        {
            comp_eval_norm[i][0] = comp_eval[i][0] / yaw_head_max;
            comp_eval_norm[i][1] = comp_eval[i][1] / z_head_max;
            comp_eval_norm[i][2] = comp_eval[i][2] / dist_max;
            comp_eval_norm[i][3] = comp_eval[i][3] / vel_max;
            G[i] = (Ky * ALFA * comp_eval_norm[i][0] + Kz * ALFA * (1 - comp_eval_norm[i][1]) + BETA * comp_eval_norm[i][2] + GAMMA * comp_eval_norm[i][3]);

            if (G[i] > max_G)
            {
                max_G = G[i];
                ind = i;
            }

            // Populate a message for visual information
            DWA_visual_msg.headingYawTerm.push_back(comp_eval_norm[i][0]);
            DWA_visual_msg.headingZTerm.push_back(1 - comp_eval_norm[i][1]);
            DWA_visual_msg.minDistTerm.push_back(comp_eval_norm[i][2]);
            DWA_visual_msg.velocityTerm.push_back(comp_eval_norm[i][3]);
            DWA_visual_msg.vx.push_back(comp_eval[i][4]);
            DWA_visual_msg.w.push_back(comp_eval[i][5]);
            DWA_visual_msg.vz.push_back(comp_eval[i][6]);
            DWA_visual_msg.G.push_back(G[i]);
        }

        // Save the velocities that optimized the function
        selected_vel[0] = round(comp_eval[ind][4] / vx_step) * vx_step; 
        selected_vel[2] = round(comp_eval[ind][6] / vz_step) * vz_step; 
        selected_vel[1] = round(comp_eval[ind][5] / w_step) * w_step; 

        // Publish command
        cmd_vel.linear.x = std::min(selected_vel[0],
                                    (w_max - fabs(current_vel[1])) / w_max * vx_max);
        
        if (cmd_vel.linear.x < 0)
        {
            cmd_vel.linear.x = 0;
        }
        cmd_vel.angular.z = selected_vel[1];
        cmd_vel.linear.z = selected_vel[2]; 
        vel_pub.publish(cmd_vel);

        // Populate a message for visual information
        DWA_visual_msg.G_selected = max_G;
        DWA_visual_msg.vx_selected = selected_vel[0];
        DWA_visual_msg.w_selected = selected_vel[1];
        DWA_visual_msg.vz_selected = selected_vel[2];
        DWA_visual_pub.publish(DWA_visual_msg);

        // Time profiling
        time_end_ = ros::WallTime::now();
        double execution_time = (time_end_ - time_start_).toNSec() * 1e-6;
        ROS_INFO_STREAM("My DWA (raycasting) computation time: " << execution_time);
	    std_msgs::Float32 time_msg;
	    time_msg.data = execution_time;
	    comp_time_pub.publish(time_msg);

        // DEBUG and RVIZ messages
        //Visualize selected velocity in RVIZ
        vel_visual_msg.twist = cmd_vel;
        vel_visual_msg.header.stamp = ros::Time::now();
        vel_visual_msg.header.frame_id = "base_link";
        vel_visual_pub.publish(vel_visual_msg);

        //Predicted pose after applying the selected velocity during delta_t
        pose_predicted = simubot(selected_vel[0], selected_vel[1], selected_vel[2], delta_t); // iter_obs*T
        visualization_msgs::Marker pose_debug;
        pose_debug.pose = pose_predicted;
        //TO-DO: Change the sphere by the UAV 3D model
        // pose_debug.type = visualization_msgs::Marker::MESH_RESOURCE;
        // pose_debug.mesh_resource = "package://hector_quadrotor_description/meshes/quadrotor/quadrotor_base.dae";
        pose_debug.type = visualization_msgs::Marker::SPHERE;
        pose_debug.action = visualization_msgs::Marker::MODIFY;
        pose_debug.color.a = 0.6;
        pose_debug.color.g = 1;
        pose_debug.scale.x = 0.1;
        pose_debug.scale.y = 0.1;
        pose_debug.scale.z = 0.1;
        pose_debug.header.frame_id = "odom";
        pose_debug.header.stamp = ros::Time::now();
        predicted_pose_pub.publish(pose_debug);

        //Casted rays for searching obstacles in the predicted pose
        populateRaysVisualMsg(pose_predicted, octomap, selected_vel[0], selected_vel[2], &casted_rays_markers); 
        markers_debug_pub.publish(casted_rays_markers);

    }
    else //No safe velocity was found!!
    {  
        vel_pub.publish(empty);
    }

    double d_robot_goal = goalDistance(current_pose, subgoal);
    /******************************************************** DEBUG INFO *****************************************************/
    if ((iteraciones % 10) == 0)
    {
        std::cout << "Goal index: " << goal_i + 1 << " | Trajectory size: " << trajectory.size() << std::endl;
        std::cout << "Goal distance: " << d_robot_goal << std::endl;
        if (goal_i == trajectory.size() - 1)
            std::cout << "ULTIMO OBJETIVO" << std::endl;
        tf::Quaternion q1(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
        tf::Matrix3x3 m1(q1);
        double roll, pitch, yaw;
        m1.getRPY(roll, pitch, yaw);
    }
    /*************************************************************************************************************************/

    //Select waypoint to follow
    if (goal_i == trajectory.size() - 1)
    {   // Ultimo objetivo
        /*                     if (d_robot_goal < 1){ // Cerca del objetivo final
                                NEAR = 0.7;
                                ALFA=0.5;
                                BETA=0.25;
                                GAMMA=0.25;
                            }  */
        if (d_robot_goal < step)
        { // Consideramos objetivo
            ROS_INFO_STREAM("GOAL REACHED");
            // Land
            bool landed = false;
            do
            {
                landed = land();
            } while (!landed);
            ROS_INFO("LANDED");
            // Disarm
            bool disarmed = false;
            do
            {
                disarmed = disarm();
            } while (!disarmed);
            ROS_INFO("DISARMED");
        }
        // Select the next subgoal if is needed
    }
    else
    {
        double d_robot_nextgoal = goalDistance(current_pose, trajectory[goal_i + 1].position);
        double heading_yaw_current = calcYawHeading(current_pose, subgoal);
        double heading_yaw_nextgoal = calcYawHeading(current_pose, trajectory[goal_i + 1].position);
        if (d_robot_goal < subgoal_step || d_robot_nextgoal < d_robot_goal || (heading_yaw_current < 0.5 && heading_yaw_nextgoal > 0.5))
        {
            do
            {
                goal_i++;
                subgoal = trajectory[goal_i].position;
                d_robot_goal = d_robot_nextgoal;
                heading_yaw_current = heading_yaw_nextgoal; 
                if (goal_i < trajectory.size() - 1)
                {
                    d_robot_nextgoal = goalDistance(current_pose, trajectory[goal_i + 1].position);
                    heading_yaw_nextgoal = calcYawHeading(current_pose, trajectory[goal_i + 1].position);
                }
            } while ((d_robot_goal < subgoal_step || d_robot_nextgoal < d_robot_goal || 
                    (heading_yaw_current < 0.5 && heading_yaw_nextgoal > 0.5)) 
                    && goal_i < trajectory.size() - 1);
            std::cout << "------------ NEXT subgoal: x=" << subgoal.x << "//y=" << subgoal.y << "//z=" << subgoal.z << " ------------" << std::endl;
        }
    iteraciones++;
    ros::spinOnce();
    loop_rate.sleep();
    }
}

void Dwa3d::idle()
{
    ros::Rate control_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        control_rate.sleep();
        if (current_state.mode == "OFFBOARD" && pose_recieved)
        {
            if (executing && octomap_recieved)
            {
                followPlan();
            }else if(!octomap_recieved){
                ROS_WARN("Waiting to recieve octomap");
            }
        }
    }
}

void Dwa3d::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // TO-DO Make it pose stamped instead of just pose to keep the header
    current_pose = msg->pose;
    pose_recieved = true;
}

void Dwa3d::currentVelCallback(const geometry_msgs::TwistStamped msg)
{
    current_vel_msg = msg.twist;
    current_vel_recieved = true;
}


/*
    Function that is in charge of retrieving the octomap from the server
    and performing the needed transformations.
*/
void Dwa3d::octomapCallback(octomap_msgs::Octomap msg){
    last_octomap_msg = msg;
    octomap_recieved = true;
}

octomap::OcTree *Dwa3d::getOctomap()
{
    return (octomap::OcTree *)octomap_msgs::msgToMap(last_octomap_msg);
}

double Dwa3d::norm_rad(double rad)
{ // rad en (-pi,pi]
    while (rad <= -PI)
        rad += 2 * PI; // (-pi, inf)
    while (rad > PI)
        rad -= 2 * PI; // (-pi, pi]
    return rad;
}

double Dwa3d::rad2deg(double rad) { return rad * 180 / PI; }

/*
    Computes the distance to the nearest voxel of the Octotree given a pose
*/
double Dwa3d::minDistOctomap(geometry_msgs::Pose last_pose, geometry_msgs::Pose predicted_pose, octomap::OcTree *octomap, double vx, double vz)
{
    double min_dist = r_search;
    double azimuth, elevation, dist, dx, dy, dz, roll0, pitch0, yaw0, x0, y0, z0, roll1, pitch1, yaw1, x1, y1, z1, roll, pitch, yaw;

    x1 = predicted_pose.position.x;
    y1 = predicted_pose.position.y;
    z1 = predicted_pose.position.z;
    yaw1 = predicted_pose.orientation.z;
    pitch1 = predicted_pose.orientation.x;
    roll1 = predicted_pose.orientation.y;


    octomap::point3d origin = octomap::point3d(x1, y1, z1);
    octomap::point3d d, ray, end;
    double xr, yr, zr;

    // TO-DO: If there is already a voxel that is closer to the drone than a certain threshold
    //  we can stop the search and keep that one as the minimum distance
    // Cast rays inside a cone to search for collisions
        double v_angle;
        v_angle = atan2(vz, vx);
    for (int i = -round(psi_beam_max / delta_psi); i < round(psi_beam_max / delta_psi); i++)
    {
        azimuth = i * delta_psi;
        // To give more importance to the obstacle in the movement direction
        double d_search_azimuth = r_search * (1 - lambda_psi * fabs(azimuth) / 1.57);
        azimuth += yaw1; // Working in global coordinates
        double cos_azi = cos(azimuth), sin_azi = sin(azimuth);
        for (int j = -round(theta_beam_max / delta_theta); j < round(theta_beam_max / delta_theta); j++)
        {
            dist = r_search;
            elevation = j * delta_theta;
            double d_search = d_search_azimuth * (1 - lambda_theta * fabs(elevation)/theta_beam_max);
            elevation += v_angle;
            double cos_elev = cos(elevation), sin_elev = sin(elevation);
            xr = cos_azi * cos_elev;
            yr = cos_elev * sin_azi;
            zr = sin_elev;

            ray = octomap::point3d(xr, yr, zr);
            ray.normalize();

            bool occupied = octomap->castRay(origin, ray, end, !treat_unknown_as_occupied, d_search);
            bool unknown = false;
            if(!occupied && treat_unknown_as_occupied){
                unknown = (octomap->search(end) == NULL);
            }
            if (occupied || unknown)
            { // True if impact an occupied voxel
                dist = origin.distance(end);
                if (dist < min_dist)
                {
                    min_dist = dist;
                }
            }
        }
    }
    min_dist -= R_drone;
    if (min_dist < 0)
    {
        min_dist = 0;
    }
    return min_dist;
}

// Alineación entre la dirección de velocidad evaluada y la dirección del objetivo en el plano XY.
double Dwa3d::calcYawHeading(geometry_msgs::Pose pose, geometry_msgs::Point goal)
{
    double diff_x = goal.x - pose.position.x, diff_y = goal.y - pose.position.y, yaw = -2;
    if (round(diff_x) == 0)
    {
        if (round(diff_y) == 0)
            yaw = 0; // Estoy en el goal --> puntuacion maxima?
        else if (round(diff_y) < 0)
            yaw = -PI / 2; // DERECHA
        else
            yaw = PI / 2; // IZQUIERDA goal.y > robot.y
    }
    else if (round(diff_y) == 0)
    { // goal.x != robot.x
        if (round(diff_x) > 0)
            yaw = 0; // DELANTE
        else
            yaw = PI; // DETRAS goal.x < robot.x
    }
    else
        yaw = atan2(diff_y, diff_x);          // goal.x != robot.x && goal.y != robot.y --> rango (-PI,PI]
    yaw -= pose.orientation.z;                // yaw: orientacion del goal respecto a la pos(x,y) del robot --> Restar la orientacion del robot
    return ((PI - fabs(norm_rad(yaw))) / PI); // * ((PI - fabs(norm_rad(yaw)))/PI); // rango [0,1]
}

// Alineacion en altura con el objetivo
double Dwa3d::calcZHeading(geometry_msgs::Pose pose, geometry_msgs::Point goal) { return goal.z - pose.position.z; }

// Modulo del vector distancia entre pose y goal
double Dwa3d::goalDistance(geometry_msgs::Pose pose, geometry_msgs::Point goal)
{
    double x_diff = goal.x - pose.position.x, y_diff = goal.y - pose.position.y, z_diff = goal.z - pose.position.z;
    double euc = sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
    return euc;
}

// Posición predicha tras ejecutar una trayectoria con velocidad [vx,vy,vz] durante un tiempo [dt]
geometry_msgs::Pose Dwa3d::simubot(double vx, double w, double vz, double dt)
{
    geometry_msgs::Pose pose_final;
    tf::Quaternion q1(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
    tf::Matrix3x3 m1(q1);
    double roll, pitch, yaw;
    m1.getRPY(roll, pitch, yaw);
    pose_final.orientation.z = norm_rad(yaw + w * dt);
    pose_final.position.x = current_pose.position.x + vx * dt * cos(pose_final.orientation.z);
    pose_final.position.y = current_pose.position.y + vx * dt * sin(pose_final.orientation.z);
    pose_final.position.z = current_pose.position.z + vz * dt;
    double v = sqrt(vx * vx + vz * vz);
    if (vz == 0 && v == 0)
        pose_final.orientation.x = 0;
    else
        pose_final.orientation.x = pitch;
    pose_final.orientation.y = roll;
    return pose_final;
}

bool Dwa3d::tryOffboard(void)
{
    bool done;
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);
    // Populate the cmd data
    cmd.linear.x = 0;
    cmd.linear.y = 0;
    cmd.linear.z = 0.1;
    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = 0;
    // cmd.header.frame_id = cmd_frame;

    // Stablish the cmd_vel frame
    mavros_msgs::SetMavFrame set_frame_msg;
    set_frame_msg.request.mav_frame = set_frame_msg.request.FRAME_BODY_NED;
    set_cmd_vel_frame.call(set_frame_msg);
    // Publish a set of commands to enable offboard
    /*
    TO-DO Check if the "for" is needed as we have
    the node from velocity_command.cpp
    that publishes the message at a
    certain rate
     */
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        vel_pub.publish(cmd);
        ros::spinOnce();
        rate.sleep();
    }

    // Pass offboard
    mavros_set_mode.request.custom_mode = "OFFBOARD";
    if (set_mode_client.call(mavros_set_mode) &&
        mavros_set_mode.response.mode_sent &&
        current_state.armed)
    {
        ROS_INFO("Offboard enabled");
        done = true;

        // Wait and hold the position until there is a plan
        cmd.linear.x = 0;
        cmd.linear.y = 0;
        cmd.linear.z = 0;
        cmd.angular.x = 0;
        cmd.angular.y = 0;
        cmd.angular.z = 0;
        // cmd.header.frame_id = cmd_frame;
        vel_pub.publish(cmd);
    }
    else
    {
        ROS_INFO("NOT ABLE TO PASS OFFBOARD");
        if (!current_state.armed)
        {
            ROS_INFO("NOT ARMED, ARM FIRST");
        }
    }
    return done;
}

bool Dwa3d::land(void)
{
    /*
    mavros_set_mode.request.custom_mode = "AUTO.LAND";
    ROS_INFO("TRYING TO PASS TO LAND MODE");
    if( set_mode_client.call(mavros_set_mode) && mavros_set_mode.response.mode_sent){
        ROS_INFO("MAVROS in LANDING MODE");
        done = true;
    }

    return done;

*/
    cmd.linear.x = 0;
    cmd.linear.y = 0;
    cmd.linear.z = -0.2;
    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = 0;
    vel_pub.publish(cmd);

    return current_extended_state.landed_state == current_extended_state.LANDED_STATE_ON_GROUND;
}

bool Dwa3d::disarm(void)
{
    if (current_state.mode == "OFFBOARD")
    {
        cmd.linear.x = 0;
        cmd.linear.y = 0;
        cmd.linear.z = 0;
        cmd.angular.x = 0;
        cmd.angular.y = 0;
        cmd.angular.z = 0;
        vel_pub.publish(cmd);
    }
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false;
    arming_client.call(arm_cmd);
    return arm_cmd.response.success;
}

void Dwa3d::initMarkers(visualization_msgs::Marker* discarded_poses_debug, 
                        visualization_msgs::Marker* casted_rays_markers){
    discarded_poses_debug->type = visualization_msgs::Marker::SPHERE_LIST;
    discarded_poses_debug->action = visualization_msgs::Marker::MODIFY;
    discarded_poses_debug->color.a = 0.2;
    discarded_poses_debug->color.r = 1;
    discarded_poses_debug->color.b = 1;
    discarded_poses_debug->scale.x = 0.1;
    discarded_poses_debug->scale.y = 0.1;
    discarded_poses_debug->scale.z = 0.1;
    discarded_poses_debug->header.frame_id = "odom";

    
    casted_rays_markers->header.frame_id = "odom";
    casted_rays_markers->type = visualization_msgs::Marker::LINE_LIST;
    // casted_rays_markers.type = visualization_msgs::Marker::CUBE_LIST;
    casted_rays_markers->action = visualization_msgs::Marker::ADD;
    casted_rays_markers->color.a = 0.6;
    casted_rays_markers->color.r = 1;
    casted_rays_markers->scale.x = 0.01;
    casted_rays_markers->scale.y = 0.01;
    casted_rays_markers->scale.z = 0.01;
    casted_rays_markers->pose.orientation.x = 0;
    casted_rays_markers->pose.orientation.y = 0;
    casted_rays_markers->pose.orientation.z = 0;
    casted_rays_markers->pose.orientation.w = 1;
}


/*
void Dwa3d::resetMarkersContents(){
    discarded_poses_debug = new visualization_msgs::Marker();
    casted_rays_markers = new visualization_msgs::Marker();
}
*/

void Dwa3d::initDwaVisualMsg(tyrion_dwa::DynamicWindowMsg* DWA_visual_msg,
                    const std::array<double, 6>& Vsd, 
                    double filas_eval, double filas_tot){
    DWA_visual_msg->paso_v = vx_step;
    DWA_visual_msg->paso_w = w_step;
    DWA_visual_msg->Vs_x_min = Vs[0];
    DWA_visual_msg->Vs_x_max = Vs[1];
    DWA_visual_msg->Vs_w_min = Vs[2];
    DWA_visual_msg->Vs_w_max = Vs[3];
    DWA_visual_msg->Vs_z_min = Vs[4];
    DWA_visual_msg->Vs_z_max = Vs[5];

    DWA_visual_msg->Vd_x_min = Vsd[0];
    DWA_visual_msg->Vd_x_max = Vsd[1];
    DWA_visual_msg->Vd_w_min = Vsd[2];
    DWA_visual_msg->Vd_w_max = Vsd[3];
    DWA_visual_msg->Vd_z_min = Vsd[4];
    DWA_visual_msg->Vd_z_max = Vsd[5];
    DWA_visual_msg->Vc = current_vel_msg;
    DWA_visual_msg->filas_eval = filas_eval;
    DWA_visual_msg->filas_tot = filas_tot;
}

void Dwa3d::populateRaysVisualMsg(geometry_msgs::Pose predicted_pose, octomap::OcTree *octomap, 
                                double vx, double vz, visualization_msgs::Marker* casted_rays_markers){
    double min_dist = r_search;
    double r, elevation, azimuth, roll1, pitch1, yaw1, x1, y1, z1;

    x1 = predicted_pose.position.x;
    y1 = predicted_pose.position.y;
    z1 = predicted_pose.position.z;
    yaw1 = predicted_pose.orientation.z;
    pitch1 = predicted_pose.orientation.x;
    roll1 = predicted_pose.orientation.y;

    octomap::point3d origin = octomap::point3d(x1, y1, z1);
    geometry_msgs::Point point0;
    point0.x = x1;
    point0.y = y1;
    point0.z = z1;
    octomap::point3d d, ray, end;
    double xr, yr, zr;

    // Cast rays inside a cone to search for collisions
    double v_angle;
    v_angle = atan2(vz, vx);
    //std::cout << "V angle: " << v_angle << std::endl;
    for (int i = -round(psi_beam_max / delta_psi); i < round(psi_beam_max / delta_psi); i++)
    {
        azimuth = i * delta_psi;
        // To give more importance to the obstacle in the movement direction
        double d_search_azimuth = r_search * (1 - lambda_psi * fabs(azimuth) / 1.57);

        azimuth += yaw1; // Working in global coordinates
        double cos_azi = cos(azimuth), sin_azi = sin(azimuth);
        for (int j = -round(theta_beam_max / delta_theta); j < round(theta_beam_max / delta_theta); j++)
        {
            elevation = j * delta_theta;
            double d_search = d_search_azimuth * (1 - lambda_theta * fabs(elevation)/theta_beam_max);
            elevation += v_angle;

            //Ray vector coordinates in a robocentric reference
            double cos_elev = cos(elevation), sin_elev = sin(elevation);
            xr = cos_azi * cos_elev;
            yr = cos_elev * sin_azi;
            zr = sin_elev;
            ray = octomap::point3d(xr, yr, zr);
            ray.normalize();
                
            //Cast ray and check occupancy
            bool occupied = octomap->castRay(origin, ray, end, !treat_unknown_as_occupied, d_search);
            bool unknown = false;
            if(!occupied && treat_unknown_as_occupied){
                unknown = (octomap->search(end) == NULL);
            }
            if (occupied || unknown)
            { // True if impact an occupied voxel
                auto xc = end.x(), yc = end.y(), zc = end.z();
                geometry_msgs::Point point;
                point.x = xc;
                point.y = yc;
                point.z = zc;
                casted_rays_markers->points.push_back(point0);
                casted_rays_markers->points.push_back(point);
                std_msgs::ColorRGBA color;
                color.a = 1;
                color.r = 1;
                color.b = 0;
                color.g = 0;
                casted_rays_markers->colors.push_back(color);
                casted_rays_markers->colors.push_back(color);
            }else{
                geometry_msgs::Point point;
                point.x = origin.x() + ray.x() * d_search;
                point.y = origin.y() + ray.y() * d_search;
                point.z = origin.z() + ray.z() * d_search;
                casted_rays_markers->points.push_back(point0);
                casted_rays_markers->points.push_back(point);
                std_msgs::ColorRGBA color;
                color.a = 1;
                color.r = 1;
                color.b = 1;
                color.g = 1;
                casted_rays_markers->colors.push_back(color);
                casted_rays_markers->colors.push_back(color);
            }
        }
    }
}