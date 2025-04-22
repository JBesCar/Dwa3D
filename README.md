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

-For Optitrack Localization, install vrpn_ros: \
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
This package is in charge of performing the reactive navigation. For deeper information refer to https://arxiv.org/abs/2409.05421

<h4> Params </h4>

<h4> Subscribed topics </h4>

<h4> Published topics </h4>
<h3> tyrion_octomap_global_planning </h3>

<h2> Third Party Packages (with modifications) </h2>


<h2> Deployment instructions </h2>
