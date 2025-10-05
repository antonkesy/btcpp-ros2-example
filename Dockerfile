FROM ubuntu:22.04 AS base

ENV TZ=Europe
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /ros2_ws
VOLUME /ros2_ws

# dependencies & nice2have
RUN apt update && \
  apt-get install -y software-properties-common && \
  apt update -y && \
  add-apt-repository -y universe && \
  apt-get install -y python3-pip wget git build-essential libboost-all-dev && \
  # clang-tools
  apt-get install -y clang-tidy clang-format && \
  # webots connection test
  apt install -y inetutils-ping netcat iproute2 && \
  # user
  apt install -y sudo

# add user
RUN useradd -ms /bin/bash user
RUN touch /home/user/.bashrc
RUN usermod -aG sudo,dialout user
RUN echo 'user ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Install ROS 2:Humble
ENV ROS_DISTRO=humble
RUN apt update && \
  apt install -y curl gnupg2 lsb-release && \
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update && \
  apt install -y ros-humble-desktop && \
  rm -rf /var/lib/apt/lists/*
EXPOSE 11311
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
# hide build warnings
RUN pip install setuptools==58.2.0

# Source ROS workspaces if exists
RUN echo "[ -e /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash" >> /root/.bashrc

# docker Webots
RUN add-apt-repository universe && \
  add-apt-repository multiverse
RUN apt update
RUN apt install -y --fix-broken wget ffmpeg libfreeimage3 libssh-dev libzip-dev xserver-xorg-core libxcb-cursor0
RUN wget -O /tmp/webots.deb https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb && apt install -y /tmp/webots.deb && \
  apt install ros-humble-webots-ros2 -y --fix-missing
EXPOSE 12345
ENV WEBOTS_HOME=/usr/local/webots
ENV LD_LIBRARY_PATH=$WEBOTS_HOME/lib/controller

# Project dependencies
RUN apt-get update --fix-missing
RUN apt install python3-colcon-common-extensions -y && \
  pip install rosdep

# Groot
RUN apt install -y qtbase5-dev qt5-qmake cmake libqt5svg5-dev
EXPOSE 5555

# First time ROS setup
RUN rosdep init && rosdep update

ENV DEBIAN_FRONTEND=newt

RUN echo "cd /ros2_ws" >> /root/.bashrc

# install dependencies on first start
RUN echo "[ -e /tmp/dependencies.lock ] || { touch /tmp/dependencies.lock && command -v make dependencies; }" >> /root/.bashrc
# build workspace on first start
RUN echo "[ -e /tmp/build.lock ] || { touch /tmp/build.lock && command -v make && make build; }" >> /root/.bashrc

# Install dependencies into container
WORKDIR /ros2_ws
COPY ./ros2_ws /ros2_ws
RUN rm log install build -rf
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && make dependencies build"
# remove build artefacts to avoid conflicts/ghots with host mount
RUN rm /ros2_ws -rf
# just to be sure :)
RUN apt-get update --fix-missing
WORKDIR /ros2_ws

FROM base AS ci
COPY ./ros2_ws /ros2_ws
WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && make dependencies && make build"
