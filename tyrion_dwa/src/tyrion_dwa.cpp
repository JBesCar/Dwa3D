#include <tyrion_dwa.h>

#define PI 3.14159265
/*
const double T = 0.1; //Periodo de control [s]
const double vx_max = 0.3; //Default = 1
const double vz_max = 0.3; //Default = 0.5
const double w_max = PI/4; // 30º --> 15º ??, Default = PI/9
const double radio_dron = 0.4; //[m], Default = 0.5
const double r_search = 1.5; // [m]
const double velodyne_max_range = 10; //Rango máximo de sensor velodyne [m]. Es el mismo definido en VLP-16.urdf.xacro, Default = 10
double res_azimuth = 90 * PI/180; //Resolución angular [°] en azimuth 30
double res_elevation = 20 * PI/180; //Resolución angular [°] en elevation 30
double resolution = 10 * PI/180;
const double paso_v = 0.05; //Resolución de discretización del espacio de búsqueda en xz[m/s], Default = 0.05
const double paso_w = PI/72; // Resolucion de discretizacion del espacio de busqueda en yaw (5º), Default = Pi/36
const double aLin = 1; // Aceleracion lineal máxima [m/ss], Default = 1.0
const double aAng = PI/1.8; // Aceleracion angular maxima [rad/ss] 10º, Default = Pi/1.8
const int cols = 8;
const int filas_tot = ((2*aLin*T/paso_v) + 1)*((2*aAng*T/paso_w) + 1)*((2*aLin*T/paso_v) + 1);

 */

Dwa3d::Dwa3d(const ros::NodeHandle &nh,
             const ros::NodeHandle &nh_private) : tf_listener_(tf_buffer_),
                                                  nh_(nh), nh_private_(nh_private)
{
    // Load parameters from ros server
    nh_.param("/ALFA", ALFA, 0.3);
    nh_.param("/Ky", Ky, 0.5);
    nh_.param("/Kz", Kz, 0.5);
    nh_.param("/BETA", BETA, 0.6);
    nh_.param("/GAMMA", GAMMA, 0.1);
    nh_.param("/goal_step", step, 0.1);
    nh_.param("/subgoal_step", subgoal_step, 0.5);
    nh_.param("iter_update", iter_update, 150);
    nh_.param("iter_obs", iter_obs, 10);
    nh_.param("cmd_vel_control_topic", cmd_vel_control_topic, std::string("/cmd_vel_control"));
    nh_.param("ground_truth_topic", ground_truth_topic, std::string("/optitrack/pose"));
    nh_.param("cmd_frame_id", cmd_frame, std::string("odom"));
    nh_.param("/r_search", r_search);
    // nh_.param("VX_MAX", vx_max);
    // nh_.param("VZ_MAX", vz_max);
    // nh_.param("W_MAX", w_max);
    nh_.param("R_drone", radio_dron);
    // nh_.param("LIDAR_MAX_RANGE",velodyne_max_range);
    nh_.param("AZIMUTH_SEARCH_RANGE", res_azimuth);
    nh_.param("ELEVATION_SEARCH_RANGE", res_elevation);

    ROS_INFO("Params loaded");
    std::cout << "ALPHA: " << ALFA << std::endl;
    std::cout << "BETA: " << BETA << std::endl;
    std::cout << "GAMMA: " << GAMMA << std::endl;

    // Initialize servers and topics
    empty.linear.x = 0;
    empty.linear.y = 0;
    empty.linear.z = 0;
    empty.angular.x = 0;
    empty.angular.y = 0;
    empty.angular.z = 0;
    vel_pub = nh_.advertise<geometry_msgs::Twist>(cmd_vel_control_topic, 10);
    vel_visual_pub = nh_.advertise<geometry_msgs::TwistStamped>("/vel_visual", 10);
    markers_debug_pub = nh_.advertise<visualization_msgs::Marker>("/markers_debug", 10);
    predicted_pose_pub = nh_.advertise<visualization_msgs::Marker>("/predicted_pose", 10);
    discarded_poses_pub = nh_.advertise<visualization_msgs::Marker>("/discarded_poses_marker", 10);
    DWA_visual_pub = nh_.advertise<tyrion_dwa::DynamicWindowMsg>("/DWA_visual_msg", 10);

    pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>(ground_truth_topic, 10, &Dwa3d::poseCallback, this);
    plan_sub = nh_.subscribe<geometry_msgs::PoseArray>("/waypoint_list", 10, &Dwa3d::planCallback, this);
    state_sub = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &Dwa3d::state_cb, this);
    current_vel_sub = nh_.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_body", 10, &Dwa3d::currentVelCallback, this);
    extended_state_sub = nh_.subscribe<mavros_msgs::ExtendedState>("mavros/extended_state", 10, &Dwa3d::extended_state_cb, this);
    octomap_sub = nh_.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, &Dwa3d::octomapCallback, this);

    arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    set_cmd_vel_frame = nh_.serviceClient<mavros_msgs::SetMavFrame>("/mavros/setpoint_velocity/mav_frame");
    octomap_server = nh_.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary", true);
    ROS_INFO("Servers and topics created");

    success = true;
    executing = false;

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
    std::array<std::array<double, cols>, filas_tot> comp_eval, comp_eval_norm; // Para cada posible velocidad de la ventana
    std::array<double, filas_tot> G;                                           // Puntuaciones
    std::array<double, 6> Vd, Vsd;
    geometry_msgs::Pose pose_predicted;
    static std::array<double, 3> current_vel = {0, 0, 0};
    std::array<double, 3> selected_vel = {0, 0, 0};
    ros::Rate loop_rate(1 / T);
    static int iteraciones = 0, iter2update = 0;
    double headingTime = 0.0, distTime = 0.0, goalDistTime = 0.0;
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
    if (current_vel_recieved)
    {
        current_vel_recieved = false;
        current_vel[0] = current_vel_msg.linear.x;
        current_vel[1] = current_vel_msg.angular.z;
        current_vel[2] = current_vel_msg.linear.z;
    }
    int filas_eval = 0, filas_x = 0, filas_w = 0, filas_z = 0; // Filas evaluadas de la ventana
    Vd = {std::max(0.0, current_vel[0]) - aLin * T, std::max(0.0, current_vel[0]) + aLin * T,
          current_vel[1] - aAng * T, current_vel[1] + aAng * T,
          current_vel[2] - aLin * T, current_vel[2] + aLin * T}; // Espacio ventana dinámica
    Vsd = {std::max(Vs[0], Vd[0]), std::min(Vs[1], Vd[1]),
           std::max(Vs[2], Vd[2]), std::min(Vs[3], Vd[3]),
           std::max(Vs[4], Vd[4]), std::min(Vs[5], Vd[5])}; // Vsd = intersección Vs con Vd
    // Inicializacion de los vectores
    for (int i = 0; i < filas_tot * cols; i++)
        comp_eval[i / cols][i % cols] = 0;
    for (int i = 0; i < filas_tot; i++)
        G[i] = 0;

    // Retrieve the Octomap from the server
    if (lecturaObstaculos)
    {
        octomap = getOctomap();
        lecturaObstaculos = false;
    }

    visualization_msgs::Marker discarded_poses_debug;
    discarded_poses_debug.type = visualization_msgs::Marker::SPHERE_LIST;
    discarded_poses_debug.action = visualization_msgs::Marker::MODIFY;
    discarded_poses_debug.color.a = 0.2;
    discarded_poses_debug.color.r = 1;
    discarded_poses_debug.color.b = 1;
    discarded_poses_debug.scale.x = 0.1;
    discarded_poses_debug.scale.y = 0.1;
    discarded_poses_debug.scale.z = 0.1;
    discarded_poses_debug.header.frame_id = "odom";

    visualization_msgs::Marker voxmap_dwa;
    voxmap_dwa.header.frame_id = "odom";
    voxmap_dwa.type = visualization_msgs::Marker::LINE_LIST;
    // voxmap_dwa.type = visualization_msgs::Marker::CUBE_LIST;
    voxmap_dwa.action = visualization_msgs::Marker::ADD;
    voxmap_dwa.color.a = 0.6;
    voxmap_dwa.color.r = 1;
    voxmap_dwa.scale.x = 0.01;
    voxmap_dwa.scale.y = 0.01;
    voxmap_dwa.scale.z = 0.01;
    voxmap_dwa.pose.orientation.x = 0;
    voxmap_dwa.pose.orientation.y = 0;
    voxmap_dwa.pose.orientation.z = 0;
    voxmap_dwa.pose.orientation.w = 1;

    // Recorrer el espacio de búsqueda Vsd
    for (double vx = Vsd[0]; vx <= Vsd[1]; vx += paso_v)
    {
        vx = round(vx / paso_v) * paso_v; // Redondeo de las velocidades
        for (double w = Vsd[2]; w <= Vsd[3]; w += paso_w)
        { //(Vsd[3]-Vsd[3]*vx/(4*std::max(0.01,Vsd[1])))
            w = round(w / paso_w) * paso_w;
            for (double vz = Vsd[4]; vz <= Vsd[5]; vz += paso_v)
            {
                vz = round(vz / paso_v) * paso_v;
                // if(goal_i < trajectory.size()-1 && vx < 0.2 && vz < 0.0) continue;
                //  Posicion final tras aplicar las velocidades
                pose_predicted = simubot(vx, w, vz, iter_obs * T); // iter_obs*T
                // Check if the predicted position is safe
                // if(!isPoseSafe(pose_predicted, octomap)){
                //      continue;
                // }
                //  Distancia al obstaculo mas cercano (en la posicion final)
                double min_dist = minDistOctomap(current_pose, pose_predicted, octomap);
                // std::cout << "Min Dist: " <<  min_dist << std::endl;
                // end_ = ros::WallTime::now();
                // distTime += (end_ - start_).toNSec() * 1e-6;
                double v = sqrt(vx * vx + vz * vz); // Velocidad total lineal dentro de Vr (dentro de Vsd y no llevan a colision)
                double wabs = fabs(w);
                // DEBUG
                {
                    geometry_msgs::Point point;
                    point.x = pose_predicted.position.x;
                    point.y = pose_predicted.position.y;
                    point.z = pose_predicted.position.z;
                    discarded_poses_debug.points.push_back(point);
                }

                if (v <= sqrt(2 * aLin * min_dist) || BETA == 0)
                { // w no deberia en ningun caso llevar a colision!!
                    // start_ = ros::WallTime::now();
                    comp_eval[filas_eval][0] = calcYawHeading(pose_predicted, subgoal);
                    double heading_z = calcZHeading(pose_predicted, subgoal); // In meters!!
                    comp_eval[filas_eval][1] = fabs(heading_z);
                    // end_ = ros::WallTime::now();
                    // headingTime += (end_ - start_).toNSec() * 1e-6;
                    comp_eval[filas_eval][2] = min_dist;

                    comp_eval[filas_eval][4] = vx;
                    comp_eval[filas_eval][5] = w;
                    comp_eval[filas_eval][6] = vz;
                    // start_ = ros::WallTime::now();
                    comp_eval[filas_eval][7] = goalDistance(pose_predicted, subgoal);
                    if (comp_eval[filas_eval][1] < 0.5)
                    { // comp_eval[filas_eval][0] > 0.9 && wabs < 2 * paso_w
                        comp_eval[filas_eval][3] = vx / vx_max;
                    }
                    else if (heading_z > 0.2 && vz > 0)
                    {
                        comp_eval[filas_eval][3] = vz / vz_max;
                    }
                    else if (heading_z < -0.2 && vz < 0)
                    {
                        comp_eval[filas_eval][3] = vz / vz_max;
                    }
                    else
                    {
                        comp_eval[filas_eval][3] = 0;
                    }
                    if (isnan(comp_eval[filas_eval][3]))
                    {
                        std::cout << "\033[42m"
                                  << "NaN"
                                  << "\033[0m" << std::endl;
                    }
                    // end_ = ros::WallTime::now();
                    // goalDistTime += (end_ - start_).toNSec() * 1e-6;
                    filas_eval++;
                }
                filas_z++;
            }
            filas_w++;
        }
        filas_x++;
    }

    // Populate a message for visual information
    tyrion_dwa::DynamicWindowMsg DWA_visual_msg;
    DWA_visual_msg.paso_v = paso_v;
    DWA_visual_msg.paso_w = paso_w;
    DWA_visual_msg.Vs_x_min = Vs[0];
    DWA_visual_msg.Vs_x_max = Vs[1];
    DWA_visual_msg.Vs_w_min = Vs[2];
    DWA_visual_msg.Vs_w_max = Vs[3];
    DWA_visual_msg.Vs_z_min = Vs[4];
    DWA_visual_msg.Vs_z_max = Vs[5];

    DWA_visual_msg.Vd_x_min = Vsd[0];
    DWA_visual_msg.Vd_x_max = Vsd[1];
    DWA_visual_msg.Vd_w_min = Vsd[2];
    DWA_visual_msg.Vd_w_max = Vsd[3];
    DWA_visual_msg.Vd_z_min = Vsd[4];
    DWA_visual_msg.Vd_z_max = Vsd[5];
    DWA_visual_msg.Vc = current_vel_msg;
    DWA_visual_msg.filas_eval = filas_eval;
    DWA_visual_msg.filas_tot = filas_tot;

    // DEBUG
    discarded_poses_debug.header.stamp = ros::Time::now();
    discarded_poses_pub.publish(discarded_poses_debug);
    voxmap_dwa.header.stamp = ros::Time::now();

    if (filas_eval != 0)
    {

        // Cálculo de máximos en la matriz de evaluación
        double yaw_head_max, z_head_max, dist_max, vel_max;
        yaw_head_max = comp_eval[0][0];
        z_head_max = comp_eval[0][1];
        dist_max = comp_eval[0][2];
        vel_max = comp_eval[0][3];
        for (int i = 0; i < filas_eval; i++)
        {
            if (comp_eval[i][0] > yaw_head_max)
                yaw_head_max = comp_eval[i][0];
            if (comp_eval[i][1] > z_head_max)
                z_head_max = comp_eval[i][1];
            // if ( comp_eval[i][2]>dist_max ) dist_max= comp_eval[i][2];
            if (comp_eval[i][3] > vel_max)
                vel_max = comp_eval[i][3];
        }
        dist_max = r_search;
        // Avoid NaNs
        if (yaw_head_max == 0)
            yaw_head_max = 1;
        if (z_head_max == 0)
            z_head_max = 1;
        if (dist_max == 0)
            dist_max = 1;
        if (vel_max == 0)
            vel_max = 1;
        double max_G = G[0];
        int ind = 0;
        for (int i = 0; i < filas_eval; i++)
        { // Normalizacion y calculo de G (Nos quedamos con el maximo)
            comp_eval_norm[i][0] = comp_eval[i][0] / yaw_head_max;
            comp_eval_norm[i][1] = comp_eval[i][1] / z_head_max;
            comp_eval_norm[i][2] = comp_eval[i][2] / dist_max;
            comp_eval_norm[i][3] = comp_eval[i][3] / vel_max;
            G[i] = (Ky * ALFA * comp_eval_norm[i][0] + Kz * ALFA * (1 - comp_eval_norm[i][1]) + BETA * comp_eval_norm[i][2] + GAMMA * comp_eval_norm[i][3]) * (1 - NEAR) + NEAR * (2 - comp_eval_norm[i][7]) / 2;
            if (G[i] > max_G)
            {
                max_G = G[i];
                ind = i;
            }
            DWA_visual_msg.headingYawTerm.push_back(comp_eval_norm[i][0]);
            DWA_visual_msg.headingZTerm.push_back(1 - comp_eval_norm[i][1]);
            DWA_visual_msg.minDistTerm.push_back(comp_eval_norm[i][2]);
            DWA_visual_msg.velocityTerm.push_back(comp_eval_norm[i][3]);
            DWA_visual_msg.vx.push_back(comp_eval[i][4]);
            DWA_visual_msg.w.push_back(comp_eval[i][5]);
            DWA_visual_msg.vz.push_back(comp_eval[i][6]);
            DWA_visual_msg.G.push_back(G[i]);
        }
        selected_vel[0] = round(comp_eval[ind][4] / paso_v) * paso_v; // vx óptimo
        selected_vel[2] = round(comp_eval[ind][6] / paso_v) * paso_v; // vz óptimo
        selected_vel[1] = round(comp_eval[ind][5] / paso_w) * paso_w; // w óptimo

        // Populate a message for visual information
        DWA_visual_msg.G_selected = max_G;
        DWA_visual_msg.vx_selected = selected_vel[0];
        DWA_visual_msg.w_selected = selected_vel[1];
        DWA_visual_msg.vz_selected = selected_vel[2];
        DWA_visual_pub.publish(DWA_visual_msg);
        time_end_ = ros::WallTime::now();
        double execution_time = (time_end_ - time_start_).toNSec() * 1e-6;
        ROS_INFO_STREAM("My DWA (raycasting) computation time: " << execution_time);

        // DEBUG
        vel_visual_msg.twist = cmd_vel;
        vel_visual_msg.header.stamp = ros::Time::now();
        vel_visual_msg.header.frame_id = "base_link";
        vel_visual_pub.publish(vel_visual_msg);

        pose_predicted = simubot(selected_vel[0], selected_vel[1], selected_vel[2], iter_obs * T); // iter_obs*T
        visualization_msgs::Marker pose_debug;
        pose_debug.pose = pose_predicted;
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

        double min_dist = r_search, azimuth, r, elevation, dist, dx, dy, dz, roll0, pitch0, yaw0, x0, y0, z0, roll1, pitch1, yaw1, x1, y1, z1, roll, pitch, yaw;
        yaw = pose_predicted.orientation.z;
        pitch = pose_predicted.orientation.x;
        roll = pose_predicted.orientation.y;

        // double resolution = 5 * M_PI / 180; //Rad

        x0 = current_pose.position.x;
        y0 = current_pose.position.y;
        z0 = current_pose.position.z;
        yaw0 = current_pose.orientation.z;
        pitch0 = current_pose.orientation.x;
        roll0 = current_pose.orientation.y;

        x1 = pose_predicted.position.x;
        y1 = pose_predicted.position.y;
        z1 = pose_predicted.position.z;
        yaw1 = pose_predicted.orientation.z;
        pitch1 = pose_predicted.orientation.x;
        roll1 = pose_predicted.orientation.y;

        octomap::point3d origin = octomap::point3d(x1, y1, z1);
        geometry_msgs::Point point0;
        point0.x = x1;
        point0.y = y1;
        point0.z = z1;
        octomap::point3d d, ray, end;
        double xr, yr, zr;

        // Compute the movement direction (it is the axis of the cone inside which the rays are cast)
        /* d = octomap::point3d(x1 - x0, y1 - y0, z1 - z0);
        if(d.norm() > 0.01){
            d.normalize();
        }else{
            //ROS_INFO("V = 0");
            tf::Quaternion q0;
            q0.setRPY(roll0,pitch0,yaw0);
            tf::Vector3 axis = q0.getAxis();
            d = octomap::point3d(axis.x(), axis.y() , axis.z());
        } */

        // Cast rays inside a cone to search for collisions
        for (int i = -round(res_azimuth / resolution); i < round(res_azimuth / resolution); i++)
        {
            azimuth = i * resolution;
            // To give more importance to the obstacle in the movement direction
            double d_search = r_search * (1 - 0.5 * fabs(azimuth) / 1.57);
            azimuth += yaw1; // Working in global coordinates
            double cos_azi = cos(azimuth), sin_azi = sin(azimuth);
            for (int j = -round(res_elevation / resolution); j < round(res_elevation / resolution); j++)
            {
                elevation = j * resolution;
                // elevation += pitch1; //Working in global coordinates
                double cos_elev = cos(elevation), sin_elev = sin(elevation);
                xr = cos_azi * cos_elev;
                yr = cos_elev * sin_azi;
                zr = cos_azi * sin_elev;

                ray = octomap::point3d(xr, yr, zr);
                ray.normalize();
                // ray += d;

                if (octomap->castRay(origin, ray, end, false, d_search))
                { // True if impact an occupied voxel
                    auto xc = end.x(), yc = end.y(), zc = end.z();
                    geometry_msgs::Point point;
                    point.x = xc;
                    point.y = yc;
                    point.z = zc;
                    voxmap_dwa.points.push_back(point0);
                    voxmap_dwa.points.push_back(point);
                    std_msgs::ColorRGBA color;
                    color.a = 1;
                    color.r = 1;
                    color.b = 0;
                    color.g = 0;
                    voxmap_dwa.colors.push_back(color);
                    voxmap_dwa.colors.push_back(color);
                }
                else
                {
                    geometry_msgs::Point point;
                    point.x = origin.x() + ray.x() * d_search;
                    point.y = origin.y() + ray.y() * d_search;
                    point.z = origin.z() + ray.z() * d_search;
                    voxmap_dwa.points.push_back(point0);
                    voxmap_dwa.points.push_back(point);
                    std_msgs::ColorRGBA color;
                    color.a = 1;
                    color.r = 1;
                    color.b = 1;
                    color.g = 1;
                    voxmap_dwa.colors.push_back(color);
                    voxmap_dwa.colors.push_back(color);
                }
            }
        }

        markers_debug_pub.publish(voxmap_dwa);
        // Publish command
        cmd_vel.linear.x = std::min(selected_vel[0],
                                    (w_max - fabs(current_vel[1])) / w_max * vx_max);
        if (cmd_vel.linear.x < 0)
        {
            cmd_vel.linear.x = 0;
        }
        cmd_vel.angular.z = selected_vel[1];
        cmd_vel.linear.z = selected_vel[2]; //+0.12 para compensación del efecto de la gravedad

        vel_pub.publish(cmd_vel);
    }
    else
    {   // COLISION
        //   ROS_INFO_STREAM("Todas las posibles combinaciones [vx,w,vz] llevan a colision con obstaculo");
        //   std::cout<<"Espacio de búsqueda: "<<Vsd[0]<<", "<<Vsd[1]<<", "<<Vsd[2]<<", "<<Vsd[3]<<", "<<Vsd[4]<<", "<<Vsd[5]<<std::endl;
        //   std::cout << "vel x = " << vel_actual[0] << "// vel z = " << vel_actual[2] << "// w = " << vel_actual[1] << std::endl;
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
        if (d_robot_goal < subgoal_step || d_robot_nextgoal < d_robot_goal)
        {
            do
            {
                goal_i++;
                subgoal = trajectory[goal_i].position;
                d_robot_goal = d_robot_nextgoal;
                if (goal_i < trajectory.size() - 1)
                {
                    d_robot_nextgoal = goalDistance(current_pose, trajectory[goal_i + 1].position);
                }
            } while ((d_robot_goal < subgoal_step || d_robot_nextgoal < d_robot_goal) && goal_i < trajectory.size() - 1);
            std::cout << "------------ NEXT subgoal: x=" << subgoal.x << "//y=" << subgoal.y << "//z=" << subgoal.z << " ------------" << std::endl;
            iter2update = 0;
        }
    }
    /*                 } else if (iter2update >= iter_update) {// Else if se queda atascado
                        ROS_INFO_STREAM("Replanning ...");
                        result_.result_code = Result::COLLISION_IN_FRONT;
                        server_.setPreempted(result_);
                        iter2update = 0;
                        return;
                    } */

    /*
                    if(goal_i < trajectory.size()-1 && octomap->castRay(actual_point,subgoal_ray,aux_end_point,true,2*radio_dron)){ //Cast the ray to check if subgoal is reachable
                        ROS_INFO_STREAM("Replanning ...");
                        result_.result_code = Result::COLLISION_IN_FRONT;
                        server_.setPreempted(result_);
                        iter2update = 0;
                        return;
                    }
                      */
    iteraciones++;
    iter2update++;
    // ROS_INFO("Iteraciones %d", iteraciones);
    ros::spinOnce();
    loop_rate.sleep();
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
    // std::cout << "Frame Odometry : " << msg->header.frame_id << std::endl;
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
    // ROS_INFO("Getting OCtomap");
/*     octomap_msgs::GetOctomap::Request req;
    octomap_msgs::GetOctomap::Response res;
    if (octomap_server.call(req, res))
    {
        octomap_msgs::Octomap octomap_msg = res.map;
        // geometry_msgs::TransformStamped global_to_local_transform = tf_buffer_.lookupTransform("base_link",octomap_global.header.frame_id,ros::Time(0));
        // Express the octomap in the local frame
        // tf2::doTransform(octomap, octomap_local, global_to_local_transform);
        // std::cout << "Octomap frame: " << octomap_local.header.frame_id << std::endl;
        // TO-DO Aplicar una transformacion al mensaje empleando las tf antes de convertirlo a octomap

        return (octomap::OcTree *)octomap_msgs::msgToMap(octomap_msg);
    } */
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
double Dwa3d::minDistOctomap(geometry_msgs::Pose last_pose, geometry_msgs::Pose predicted_pose, octomap::OcTree *octomap)
{
    double min_dist = r_search;
    double azimuth, elevation, dist, dx, dy, dz, roll0, pitch0, yaw0, x0, y0, z0, roll1, pitch1, yaw1, x1, y1, z1, roll, pitch, yaw;
    // auto resolution = octomap->getResolution();
    x0 = last_pose.position.x;
    y0 = last_pose.position.y;
    z0 = last_pose.position.z;
    yaw0 = last_pose.orientation.z;
    pitch0 = last_pose.orientation.x;
    roll0 = last_pose.orientation.y;

    x1 = predicted_pose.position.x;
    y1 = predicted_pose.position.y;
    z1 = predicted_pose.position.z;
    yaw1 = predicted_pose.orientation.z;
    pitch1 = predicted_pose.orientation.x;
    roll1 = predicted_pose.orientation.y;

    // const double resolution = 5 * M_PI / 180; //Rad

    octomap::point3d origin = octomap::point3d(x1, y1, z1);
    octomap::point3d d, ray, end;
    double xr, yr, zr;

    // Compute the movement direction (it is the axis of the cone inside which the rays are cast)
    /* d = octomap::point3d(x1 - x0, y1 - y0 , z1 - z0);
    if(d.norm() > 0.01){
        d.normalize();
    }else{
        //ROS_INFO("V = 0");
        //tf::Quaternion q0;
        //q0.setRPY(roll0,pitch0,yaw0);
        //tf::Vector3 axis = q0.getAxis();
        d = octomap::point3d(0, 0, 0);//octomap::point3d(axis.x(), axis.y() , axis.z());
    } */

    // TO-DO: If there is already a voxel that is closer to the drone than a certain threshold
    //  we can stop the search and keep that one as the minimum distance
    // Cast rays inside a cone to search for collisions
    for (int i = -round(res_azimuth / resolution); i < round(res_azimuth / resolution); i++)
    {
        azimuth = i * resolution;
        // To give more importance to the obstacle in the movement direction
        double d_search = r_search * (1 - 0.5 * fabs(azimuth) / 1.57);
        azimuth += yaw1; // Working in global coordinates
        double cos_azi = cos(azimuth), sin_azi = sin(azimuth);
        for (int j = -round(res_elevation / resolution); j < round(res_elevation / resolution); j++)
        {
            dist = r_search;
            elevation = j * resolution;
            // elevation += atan2(z1 - z0, x1 - x0);
            // elevation += pitch1; //Working in global coordinates
            double cos_elev = cos(elevation), sin_elev = sin(elevation);
            xr = cos_azi * cos_elev;
            yr = cos_elev * sin_azi;
            zr = cos_azi * sin_elev;

            ray = octomap::point3d(xr, yr, zr);
            ray.normalize();
            // ray += d;

            if (octomap->castRay(origin, ray, end, false, d_search))
            { // True if impact an occupied voxel
                dist = origin.distance(end);
                if (dist < min_dist)
                {
                    min_dist = dist;
                }
            }
        }
        // Cast rays also towards the roof and towards the floor
        if (octomap->castRay(origin, octomap::point3d(0, 0, 1), end, false, r_search))
        { // True if impact an occupied voxel
            dist = origin.distance(end);
            if (dist < min_dist)
            {
                min_dist = dist;
            }
        }
        // if(octomap->castRay(origin,octomap::point3d(0,0,-1),end,true,r_search)){ //True if impact an occupied voxel
        //     dist = origin.distance(end);
        //     if(dist < min_dist){
        //         min_dist = dist;
        //     }
        // }
    }
    min_dist -= radio_dron;
    // min_dist -= 1.73 * resolution;
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

bool Dwa3d::isPoseSafe(geometry_msgs::Pose pose_predicted)
{
    bool is_safe = false;
    double x = pose_predicted.position.x;
    double y = pose_predicted.position.y;
    double z = pose_predicted.position.z;
    double qx = pose_predicted.orientation.x;
    double qy = pose_predicted.orientation.y;
    double qz = pose_predicted.orientation.z;
    double qw = pose_predicted.orientation.w;

    return is_safe;
}

bool Dwa3d::tryOffboard(void)
{
    bool done;
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    // Populate the cmd data
    cmd.linear.x = 0;
    cmd.linear.y = 0;
    cmd.linear.z = 0.2;
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

void Dwa3d::queryReplan()
{
    ROS_INFO("ASKING FOR REPLANNING");
    vel_pub.publish(empty);
}

/*
int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_executor");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ROS_INFO("Creating DWA_controller");
    Dwa3d controller(nh, nh_private);
    ROS_INFO("DWA_controller created");
    controller.idle();
    return 0;
}
 */