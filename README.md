1) Install tf2-sensor-msgs: \
`sudo apt-get install ros-${ROS_DISTRO}-tf2-sensor-msgs`
2) Install mavros: \
`sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs`
`wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh`
4) Install moveIt: \
`sudo apt install ros-${ROS_DISTRO}-moveit` 
5) Install vrpn_ros:
` sudo apt-get install ros-${ROS_DISTRO}-vrpn-client-ros `
6) Install octomap
`sudo apt-get install ros-noetic-octomap ros-noetic-octomap-ros`
7) Install ouster_drivers:
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
