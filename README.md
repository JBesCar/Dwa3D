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
This package is in charge of performing the reactive navigation. For deeper information about the method and the involved parameters refer to https://arxiv.org/abs/2409.05421

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
- `mavros/cmd/arming`: 
- `mavros/set_mode` :
- `mavros/setpoint_velocity/mav_frame`:

<h4> Published topics </h4>

- `/cmd_vel_control` (configurable as a param): Send velocity command to UAV, Type: `geometry_msgs::Twist`
- `predicted_pose`: Pose that would be reached if the selected velocity was applied during `delta_t`, Type: `visualization_msgs::Marker`
- `discarded_poses`: The predicted poses for the rest of velocities that have not been selected, Type: `visualization_msgs::Marker`
- `markers_debug`: Casted rays from the predicted pose reached if the selected velocity is applied during `delta_t`, Type: `visualization_msgs::Marker`
- `DWA_visual_msg`: Visual information about the Dynamic Window values at each control step. Can be visualized with the node `Dwa_Visual_Info.py` of this package. Type: `tyrion_dwa::DynamicWindowMsg`
- `dwa_computational_time`: Computational time of DWA-3D for each control step. Type: `std_msgs::Float32`

<h3> tyrion_octomap_global_planning </h3>

<h2> Third Party Packages (with modifications) </h2>


<h2> Deployment instructions </h2>
