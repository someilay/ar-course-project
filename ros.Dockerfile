# Ros distro
ARG ROS_DISTRO=humble

FROM osrf/ros:${ROS_DISTRO}-desktop-full

# Arguments
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Example of installing programs
RUN apt-get update \
    && apt-get install -y \
    nano \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

# Install additional ROS2 packages
RUN apt-get update && \
    apt-get install -y \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-ament-package \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-simple-launch \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-ign-ros2-control

# Install Pinocchio and dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-pinocchio \
    ros-${ROS_DISTRO}-hpp-fcl \
    libboost-python1.74-dev \
    python3.10-dev

# Clear apt-get cache
RUN rm -rf /var/lib/apt/lists/*

# Create a symlink for Python 3.10 headers
RUN sudo ln -sf /usr/include/python3.10 /usr/include/boost/python310

# Copy the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly
# COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

USER $USERNAME

# COPY entrypoint.sh ~/entrypoint.sh

# Set up entrypoint and default command
# ENTRYPOINT ["/bin/bash", "~/entrypoint.sh"]
# CMD ["bash"]
ENTRYPOINT ["/bin/bash"]
