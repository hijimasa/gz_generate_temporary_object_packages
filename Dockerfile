FROM ros:humble

ENV DEBIAN_FRONTEND noninteractive

# install dependencies via apt
ENV DEBCONF_NOWARNINGS yes
RUN set -x && \
  apt-get update -y -qq && \
  apt-get upgrade -y -qq --no-install-recommends && \
  apt-get install -y -qq \
    libyaml-cpp-dev \
    vim v4l-utils exfat-* \
    openssh-server \
    terminator dbus-x11 \
    python3-pip && \
  : "remove cache" && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*

# for ros2
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \
    python3-colcon-ros \
    && apt-get clean

RUN set -x && \
  apt-get update -y -qq && \
  apt-get install -y -qq ros-humble-desktop \
    ros-humble-ament-cmake ros-humble-angles ros-humble-controller-manager \
    ignition-fortress ros-humble-ros-gz* ros-humble-ros-ign* libgz-sim* python3-gz-sim6 \
    ros-humble-pluginlib ros-humble-urdf ros-humble-yaml-cpp-vendor ros-humble-joint-state-pub* \
    ros-humble-cv-bridge ros-humble-diagnostic-updater && \
  apt-get install -y -qq ros-humble-xacro ros-humble-realtime-tools ros-humble-control-toolbox ros-humble-ros2-control* && \
  : "remove cache" && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*

RUN set -x && \
  pip3 install -U pip && \
  pip3 install pyserial numpy scipy

RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
RUN echo 'export GZ_VERSION=fortress' >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]
