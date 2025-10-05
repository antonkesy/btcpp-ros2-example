# https://hub.docker.com/r/nvidia/cuda
FROM nvidia/cuda:12.3.0-runtime-ubuntu22.04 as base

ENV TZ=Europe
ENV DEBIAN_FRONTEND noninteractive

WORKDIR /ros2_ws
VOLUME /ros2_ws

# Install some dependencies
RUN apt update && \
  apt-get install software-properties-common -y && \
  apt update -y && \
  add-apt-repository universe -y && \
  apt install python3-pip -y && \
  apt-get install build-essential -y && \
  apt-get install clang-tidy clang-format -y && \
  apt install -y inetutils-ping netcat iproute2

# add user
RUN useradd -ms /bin/bash user
RUN touch /home/user/.bashrc
RUN usermod -aG sudo,dialout user
RUN echo 'user ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Install ROS 2:Humble
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
RUN apt update && \
  mkdir -p /etc/apt/keyrings && \
  apt-get install wget -y && \
  apt-get install software-properties-common -y && \
  cd /etc/apt/keyrings && \
  wget -q https://cyberbotics.com/Cyberbotics.asc && \
  echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | tee /etc/apt/sources.list.d/Cyberbotics.list && \
  apt update && \
  apt-get install webots=2023b -y && \
  apt install ros-humble-webots-ros2 -y
EXPOSE 12345
ENV WEBOTS_HOME=/usr/local/webots
ENV LD_LIBRARY_PATH=$WEBOTS_HOME/lib/controller:$LD_LIBRARY_PATH

# Project dependencies
RUN apt-get update --fix-missing
RUN apt install python3-colcon-common-extensions -y && \
  pip install rosdep

# Groot
RUN apt install -y qtbase5-dev qt5-qmake cmake libqt5svg5-dev
EXPOSE 5555

# First time ROS setup
RUN rosdep init && rosdep update

ENV DEBIAN_FRONTEND newt

RUN echo "cd /ros2_ws" >> /root/.bashrc

# install dependencies on first start
RUN echo "[ -e /tmp/dependencies.lock ] || { touch /tmp/dependencies.lock && command -v make && make dependencies; }" >> /root/.bashrc
# build workspace on first start
RUN echo "[ -e /tmp/build.lock ] || { touch /tmp/build.lock && command -v make && make build; }" >> /root/.bashrc

FROM base as ci
COPY ./ros2_ws /ros2_ws
WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && make dependencies && make build"
