FROM nvidia/cudagl:11.1.1-base-ubuntu20.04
 
# Minimal setup
RUN apt-get update \
 && apt-get install -y locales lsb-release curl
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales
 
ARG ROS_DISTRO=noetic
# Install ROS ROS_DISTRO
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update \
 && apt-get install -y ros-${ROS_DISTRO}-desktop-full
RUN apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update

ARG PROJECT=gem_path_tracking
ARG WS_PATH=/home/${PROJECT}

# Install POLARIS repo
RUN apt install -y ros-${ROS_DISTRO}-ackermann-msgs ros-${ROS_DISTRO}-geometry2 \
    ros-${ROS_DISTRO}-hector-gazebo ros-${ROS_DISTRO}-hector-models ros-${ROS_DISTRO}-jsk-rviz-plugins \
    ros-${ROS_DISTRO}-ros-control ros-${ROS_DISTRO}-ros-controllers ros-${ROS_DISTRO}-velodyne-simulator

WORKDIR ${WS_PATH}/src
RUN git clone https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git

# Install Modified POLARIS + gem_path_tracking
RUN apt install -y python3-pip
RUN pip3 install numpy
RUN pip3 install matplotlib
RUN pip3 install do-mpc
RUN apt install -y ros-${ROS_DISTRO}-robot-localization

RUN git clone https://github.com/lucaslins0035/MPC_UGV.git

WORKDIR ${WS_PATH}
RUN apt install -y ros-${ROS_DISTRO}-catkin
RUN /bin/bash -c '. /opt/ros/${ROS_DISTRO}/setup.sh \
        && catkin_make'

# Configure handy source file
RUN touch /rsource.sh
RUN echo "#!/bin/bash \n\n\
source /opt/ros/$ROS_DISTRO/setup.bash \n\n\
if [ -e $WS_PATH/devel/setup.bash ] \n\
then \n\
  source $WS_PATH/devel/setup.bash \n\
fi" >> /rsource.sh
RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /rsource.sh
RUN echo "PS1='${debian_chroot:+($debian_chroot)}\[\033[38;5;208m\]bm@${ROS_DISTRO}\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> /rsource.sh