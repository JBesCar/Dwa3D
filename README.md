<h1> Real Drone (jetson_orin branch)</h1> 

This branch contains the code used for the deployment of the real UAV.

<h2> Installation instructions </h2>

1) Install tf2-sensor-msgs: \
`sudo apt-get install ros-${ROS_DISTRO}-tf2-sensor-msgs`
2) Install mavros: \
`sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs` \
`wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh` \
`sudo bash ./install_geographiclib_datasets.sh`
3) Install octomap \
`sudo apt-get install ros-{ROS_DISTRO}-octomap ros-{ROS_DISTRO}-octomap-ros`
4) Install hector_trajectory_server for trajectory visualization: \
`sudo apt-get install ros-{ROS_DISTRO}-hector-trajectory-server`
5) Clone this branch in your catkin workspace: \
`git clone --branch jetson_orin https://github.com/JBesCar/Dwa3D`
6) Set build to release: \
` catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release `

<h3> Optional packages (according with your setup) </h3>

- For Optitrack Localization, install vrpn_ros: \
` sudo apt-get install ros-${ROS_DISTRO}-vrpn-client-ros `

- For Ouster 3D-LiDAR, install ouster_drivers requirements: \
`sudo apt install -y                     \
    ros-$ROS_DISTRO-pcl-ros             \
    ros-$ROS_DISTRO-rviz `
`sudo apt install -y         \
    build-essential         \
    libeigen3-dev           \
    libjsoncpp-dev          \
    libspdlog-dev           \
    libcurl4-openssl-dev    \
    cmake `
- For F-LOAM Localization, install Ceres solver (V2.1)
http://ceres-solver.org/installation.html


<h2> Own Packages </h2>
Documentation of each package

<h3> tyrion_dwa </h3>
This package provides the node in charge of performing the reactive navigation. For deeper information about the method and the involved parameters refer to https://arxiv.org/abs/2409.05421
There is an additional Python node, `Dwa_Visual_Info.py`, for displaying visual information about the DWA-3D decissions in the Search Space.
![DWA_COlors](https://github.com/user-attachments/assets/bee51ca7-816b-4823-80bf-9f70c536150f)
The launch file `tyrion_dwa.launch` launches the node and loads its parameters.
<h4> Params </h4>

- `R_drone`: Drone radius (m)
- `T_control`: Control period (s)
- `delta_t`: Prediction Temporal Horizon (s)
- `vx_step`: Discretization step for forward (x) velocity (m/s)
- `vz_step`: Discretization step for vertical (z) velocity (m/s)
- `w_step`: Discretization step for angular (wz) velocity (rad/s)
- `vx_max`: Maximum forward velocity (m/s)
- `vz_max`: Maximum absolute vertical velocity (m/s)
- `w_max`: Maximum absolute angular velocity (rad/s)
- `aLin`: Maximum linear acceleration (m/s²)
- `aAng`: Maximum angular acceleration (rad/s²)
- `ALFA`: 
- `BETA`:
- `GAMMA`:
- `Ky`:
- `Kz`:
- `goal_step`: Distance at which the final destination is considered as reached (m)
- `subgoal_step`: Distance at which a waypoint is considered as reached (m)
- `r_search`: Maximum distance at which the obstacles are searched (m)
- `psi_beam_max`:
- `theta_beam_max`:
- `delta_psi`:
- `delta_theta`:
- `lambda_psi`:
- `lambda_theta`:
- `treat_unknown_as_occupied`: Boolean, used to select between considering unknown areas as occupied (safer) or free (riskier) 
- `cmd_vel_control_topic`: Topic in which the cmd_vel is published to be sent to the UAV
- `pose_topic`: Subscribed topic to know about the UAV localization
- `plan_topic`: Subscribed topic to recieve the global plan
- `current_vel_topic`: Subscribed topic to recieve feedback about the current velocity



<h4> Subscribed topics </h4>

- `pose_topic` (configurable as a param): Subscribed topic to know about the UAV localization, Type: `geometry_msgs::PoseStamped`
- `plan_topic` (configurable as a param): Subscribed topic to recieve the global plan, Type: `geometry_msgs::PoseArray`
- `current_vel_topic` (configurable as a param): Subscribed topic to recieve feedback about the current velocity, Type: `geometry_msgs::TwistStamped`
- `/octomap_binary`: Topic from which the octomap is acquired, Type: `octomap_msgs::Octomap`
- `mavros/state`: Information about autopilot status, Type: `mavros_msgs::State`
- `mavros/extended_state`: More information about autopilot status, Type: `mavros_msgs::ExtendedState`

<h4> Servers Calls </h4>

- `mavros/cmd/arming`: Server to arm PX4 autopilot, Type: `mavros_msgs::CommandBool` 
- `mavros/set_mode` : Server to change the mode of the PX4 autopilot, Type `mavros_msgs::SetMode`
- `mavros/setpoint_velocity/mav_frame`: Server to specify mavros in which frame the velocity commands are expresed (`FRAME_BODY_NED`), Type: `mavros_msgs::SetMavFrame`

<h4> Published topics </h4>

- `/cmd_vel_control` (configurable as a param): Send velocity command to UAV, Type: `geometry_msgs::Twist`
- `predicted_pose`: Pose that would be reached if the selected velocity was applied during `delta_t`, Type: `visualization_msgs::Marker`
- `discarded_poses`: The predicted poses for the rest of velocities that have not been selected, Type: `visualization_msgs::Marker`
- `markers_debug`: Casted rays from the predicted pose reached if the selected velocity is applied during `delta_t`, Type: `visualization_msgs::Marker`
- `DWA_visual_msg`: Visual information about the Dynamic Window values at each control step. Can be visualized with the node `Dwa_Visual_Info.py` of this package. Type: `tyrion_dwa::DynamicWindowMsg`
- `dwa_computational_time`: Computational time of DWA-3D for each control step. Type: `std_msgs::Float32`

<h3> tyrion_octomap_global_planning </h3>
This package provides an example node to compute a global path for DWA-3D using the OMPL library. The selected planner is RRT*. 

The launch file `tyrion_rrt_octomap.launch` launches the node and loads its parameters.

<h4> Params </h4>

- `XMIN`:
- `XMAX`:
- `YMIN`:
- `YMAX`:
- `ZMIN`:
- `ZMAX`:
- `safety_distance`:
- `max_planning_time`:
- `max_segment_length`:
- `enable_replan`:
- `odom_topic`:
- `goal_topic`: 
- `octomap_topic`:
- `markers_path_topic`:
- `waypoints_topic`:

    
<h4> Subscribed topics </h4>

- `odom_topic`(configurable as a param): Subscribed topic to know about the UAV localization, Type: `geometry_msgs::PoseStamped`
- `goal_topic`(configurable as a param): Subscribed topic to recieve the goal, Type: `geometry_msgs::PoseStamped`
- `octomap_topic`(configurable as a param): Topic from which the octomap is acquired, Type: `octomap_msgs::Octomap`
  
<h4> Published topics </h4>

- `markers_path_topic`(configurable as a param): Path to be displayed in RViz, Foxglove or similar, Type: `visualization_msgs::Marker`
- `waypoints_topic`(configurable as a param): Topic to publish the global plan for DWA-3D, Type: `geometry_msgs::PoseArray`


<h2> Third Party Packages (with modifications) </h2>


<h2> Deployment instructions </h2>

1) Launch mavros, start sensors, high level nodes and related, in our case: \
`roslaunch px4_tyrion bridge_mavros_tyrionOuster.launch`
2) Launch the selected localization method, for instance: \
`roslaunch floam floam_ouster.launch ` \
or \
`roslaunch optitrack_arena tyrion_gt_optitrack.launch`

3) Launch Octomap: \
`roslaunch octomap_server octomap_mapping.launch`

4) Takeoff:\
` rostopic pub /order std_msgs/String "data: 'TAKEOFF'" ` \
or manually

6) Launch Global Planner: \
`roslaunch tyrion_octomap_global_planning tyrion_rrt_octomap.launch`
7) Send Goal: \
`rostopic pub /goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'odom'
pose:
  position:
    x: 10.0
    y: 0.0
    z: 1.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0" 
 ` \
or publish it from any of your nodes

9) Launch DWA-3D: 
`roslaunch tyrion_dwa tyrion_dwa.launch`

