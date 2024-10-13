# ----- Define base image -----
FROM nvidia/cuda:12.6.1-base-ubuntu24.04
    
# ----- Define environmental variables -----
ENV HOME=/home
ENV TZ=Europe/Oslo
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# ----- Install ROS2 Jazzy -----
RUN apt update && apt install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
RUN apt install -y software-properties-common
RUN add-apt-repository universe
RUN apt update && apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update && apt install -y ros-dev-tools
RUN apt update -y && apt upgrade -y
RUN apt install -y ros-jazzy-desktop
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# ----- Install Gazebo Harmonic -----
RUN apt-get update
RUN apt-get install -y lsb-release gnupg
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update
RUN apt-get install -y gz-harmonic

# ----- Install Gazebo/ROS pairing -----
RUN apt update -y && apt-get install -y ros-jazzy-ros-gz

# ----- Install misc tools -----
RUN apt update -y && apt install -y ros-jazzy-robot-localization

# ----- Install PlotJuggler -----
RUN apt update && apt install -y ros-jazzy-plotjuggler-ros

ENV GAZEBO_MODEL_PATH=/opt/ros/jazzy/share
ENV GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share

RUN apt update -y && apt upgrade -y

WORKDIR $HOME
        