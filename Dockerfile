FROM osrf/ros:noetic-desktop-full-focal
ENV DEBIAN_FRONTEND=noninteractive
RUN apt update
RUN apt install -y vim
RUN apt install -y wget
RUN apt install -y nano
RUN apt install -y python3-pip
RUN apt-get install -y python3-osrf-pycommon
RUN python3 -m pip install --upgrade pip
RUN apt install -y libgl1-mesa-glx mesa-utils libosmesa6
RUN apt update && apt-get install -y \
    libx11-dev \
    libglu1-mesa \
    libgl1-mesa-glx \
    libglu1-mesa \
    libdrm2 \
    xserver-xorg-video-intel \
    mesa-utils
RUN pip3 install -U catkin_tools 
RUN pip3 install --upgrade matplotlib
RUN apt-get install -y ros-noetic-tf2-sensor-msgs
RUN apt-get install -y ros-noetic-octomap ros-noetic-octomap-ros
RUN apt-get install -y ros-noetic-hector-trajectory-server
RUN apt-get install -y ros-noetic-mavros 
RUN apt-get install -y ros-noetic-mavros-extras 
RUN apt-get install -y ros-noetic-mavros-msgs
RUN apt-get install -y ros-noetic-mavlink
RUN apt-get install -y ros-noetic-ompl
RUN apt-get install -y ros-noetic-geographic-msgs
RUN apt-get install -y libgeographic-dev
RUN pip3 install kconfiglib
RUN pip3 install --user jinja2
RUN pip3 install --user jsonschema
RUN pip3 install --user pyros-genmsg
RUN pip3 install future
RUN apt-get install -y tmux
RUN cd /home && wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && bash /home/install_geographiclib_datasets.sh  
# JORGE --mount and build px4--
ADD PX4 /root/PX4
RUN bash /root/PX4/Tools/setup/ubuntu.sh --no-nuttx
# JORGE --mount and build catkin_ws-- [must be done after building lorenzo's catkin workspace]
ADD dwa_ws /home/dwa_ws
RUN . /opt/ros/noetic/setup.sh && cd /home/dwa_ws && catkin build -j1
# JORGE --mount and set px4 parameters--
#COPY params_PX4/parameters.bson /home
# JORGE --add px4 to gazebo--
WORKDIR /root/PX4
RUN DONT_RUN=1 make px4_sitl_default gazebo-classic
# JORGE --mount and set px4 parameters--
#ADD params_PX4/etc /root/PX4/build/px4_sitl_default/
#COPY params_PX4/parameters.bson /root/PX4/build/px4_sitl_default/etc/params.bson
COPY params_PX4/etc/init.d-posix/px4-rc.tyrionParams /root/PX4/build/px4_sitl_default/etc/init.d-posix/px4-rc.tyrionParams
COPY params_PX4/etc/init.d-posix/rcS /root/PX4/build/px4_sitl_default/etc/init.d-posix/rcS

# This is to hasten gazebo kill
ADD replace_string.sh /replace_string.sh
RUN chmod +x /replace_string.sh
RUN /replace_string.sh /opt/ros/noetic/lib/python3/dist-packages/roslaunch/nodeprocess.py "15.0" "0.2"
RUN /replace_string.sh /opt/ros/noetic/lib/python3/dist-packages/roslaunch/nodeprocess.py "2.0" "0.2"


COPY entrypoint.sh /entrypoint.sh
COPY source_px4.sh /source_px4.sh
RUN echo 'source /entrypoint.sh' >> /root/.bashrc

#RUN chmod +x /entrypoint.sh
## Set the entrypoint to run the script
#ENTRYPOINT ["/entrypoint.sh"]
## Optionally specify a default command (e.g., bash)
#CMD ["/bin/bash"]
