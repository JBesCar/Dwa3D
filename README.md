<h1> Simulation branch</h1> 

This branch provides the code to simulate an UAV with the same setup as the one used in the real experiments. 

A Docker with all the packages and configurations to use it is available. 


<h2> Installation instructions (Docker) </h2>

1) Make sure you have Docker installed in your system.

   https://docs.docker.com/engine/install/

2) Clone this branch in the folder you desire \
   ` git clone --branch simulation https://github.com/JBesCar/Dwa3D `

3) Execute the `build_docker.sh` file provided to build the Docker image.
4) To run the first instance of that image use the `launch_docker.sh` file.
5) If you want to execute it on more terminals you can use the `exec_docker.sh` file.

<h2> Own Packages </h2>
Documentation of each package

<h3> px4_tyrion </h3>

Provides several helpful nodes to interact with the PX4 autopilot via MAVROS and to send high level orders to the UAV.
The launch file `spawn_tyrion.launch` spawns in Gazebo an UAV ready to fly with a simulated PX4 autopilot and our autonomous navigation architecture. Feel free to customize your own one according with your setup. 


<h3> tyrion_dwa </h3>
This package provides the node in charge of performing the reactive navigation. For deeper information about the method and the involved parameters refer to https://arxiv.org/abs/2409.05421.
There is an additional Python node, `Dwa_Visual_Info.py`, for displaying visual information about the DWA-3D decissions in the Search Space.

![DWA_COlors](https://github.com/user-attachments/assets/bee51ca7-816b-4823-80bf-9f70c536150f)

The launch file `tyrion_dwa.launch` launches the navigation node and loads its parameters. Optionally it can also launch the visual displayer one.
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
- `ALFA`: Weight of the cost function that prioritizes allignment with the path.
- `BETA`: Weight of the cost function that prioritizes avoiding obstacles.
- `GAMMA`: Weight of the cost function that prioritizes high velocities.
- `Ky`: Weight of the alligment term that prioritizes staying alligned in the horizontal plane. (If Ky > Kz, vertical avoidance is preferred)
- `Kz`: Weight of the alligment term that prioritizes staying at the height of the waypoint. (If Kz > Ky, lateral avoidance is preferred)
- `goal_step`: Distance at which the final destination is considered as reached (m)
- `subgoal_step`: Distance at which a waypoint is considered as reached (m)
- `r_search`: Maximum distance at which the obstacles are searched (m)
- `psi_beam_max`: Raycasting limit angle in the XY plane (rad).
- `theta_beam_max`: Raycasting limit angle in the XZ plane (rad).
- `delta_psi`: Angular distance between rays casted in the XY plane (rad).
- `delta_theta`: Angular distance between rays casted in the XZ plane (rad).
- `lambda_psi`: Parameter to configure the lateral safety distance to `r_search * (1-lambda_psi)`.
- `lambda_theta`: Parameter to configure the vertical safety distance to `r_search * (1-lambda_theta)`.
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

- `XMIN`, `XMAX`, `YMIN`, `YMAX`, `ZMIN`, `ZMAX`: RRT* sampling limits. 
- `safety_distance`: Minimum distance allowed between the path and obstacle. If < 0 size awareness is disabled.
- `max_planning_time`: Maximum time given to the RRT* to find a solution.
- `max_segment_length`: Maximum distance between two consecutive waypoints.
- `enable_replan`: Bool, enable RRT* to search for new solutions during the navigation once it has computed the first valid one. 
- `odom_topic`: Subscribed topic to know about the UAV localization.
- `goal_topic`: Subscribed topic to recieve the goal.
- `octomap_topic`: Topic from which the octomap is acquired.
- `markers_path_topic`: Path to be displayed in RViz, Foxglove or similar.
- `waypoints_topic`: Topic from which the octomap is acquired.

    
<h4> Subscribed topics </h4>

- `odom_topic`(configurable as a param): Subscribed topic to know about the UAV localization, Type: `geometry_msgs::PoseStamped`
- `goal_topic`(configurable as a param): Subscribed topic to recieve the goal, Type: `geometry_msgs::PoseStamped`
- `octomap_topic`(configurable as a param): Topic from which the octomap is acquired, Type: `octomap_msgs::Octomap`
  
<h4> Published topics </h4>

- `markers_path_topic`(configurable as a param): Path to be displayed in RViz, Foxglove or similar, Type: `visualization_msgs::Marker`
- `waypoints_topic`(configurable as a param): Topic to publish the global plan for DWA-3D, Type: `geometry_msgs::PoseArray`


<h2> Third Party Packages (with modifications) </h2>

<h3> FLOAM </h3>

https://github.com/wh200720041/floam

<h3> octomap_mapping </h3>

https://github.com/OctoMap/octomap_mapping

<h3> ouster_ros </h3>

Drivers for Ouster 3D-LiDAR. https://github.com/ouster-lidar/ouster-ros

<h3> tfmini_ros </h3>

Drivers for tfmini RangeFinder. https://github.com/TFmini/TFmini-ROS


<h2> Deployment instructions </h2>

1) Launch Gazebo with the desired world, for instance: \
`roslaunch gazebo_ros empty_world.launch`
2) Spawn the ready to fly UAV with the provided launch in the desired coordinates: \
`roslaunch px4_tyrion spawn_tyrion.launch x:=0.0 y:=0.0 z:=0.0` \
After a couple of seconds you should have the UAV in Gazebo and a GUI with high level commands
![GUI](https://github.com/user-attachments/assets/146f8d2f-9569-4020-a5b5-e29d56b60fe0)

4) Send a Goal: \
`rostopic pub /tyrion/goal geometry_msgs/PoseStamped "header:
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

5) Takeoff:\
 Click takeoff in the GUI
![GUI_TAKEOFF](https://github.com/user-attachments/assets/0c25956f-fa17-4fc9-8459-ca3734b5638b)

or \
` rostopic pub /order std_msgs/String "data: 'TAKEOFF'" ` 

6) Once the UAV is idle, send it to NAVIGATE \
   With the GUI
   ![NAVIGATE](https://github.com/user-attachments/assets/f0911fe8-f4df-49b1-97fc-b53bf934b024)

or \
` rostopic pub /order std_msgs/String "data: 'NAVIGATE'" ` 


The rosgraph should have the following appareance

![rosgraph_simulation](https://github.com/user-attachments/assets/0151b489-44c4-4244-b9c1-aad68a37eadc)


