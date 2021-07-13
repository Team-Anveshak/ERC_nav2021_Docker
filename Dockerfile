FROM ghcr.io/europeanroverchallenge/erc-remote-image-base:latest

# Install additional packages
RUN apt-get update && apt-get -y upgrade && apt-get -y install \
  tmux \
  && rm -rf /var/lib/apt/lists/*

# Copy packages and build the workspace
WORKDIR /catkin_ws
COPY src ./src
RUN apt-get update \
  && rosdep update \
  && rosdep install --from-paths src -iy \
  && rm -rf /var/lib/apt/lists/*
RUN apt-get -y install ros-melodic-ar-track-alvar \
  ros-melodic-robot-localization \
  ros-melodic-move-base \
  ros-melodic-grid-map \
  ros-melodic-topic-tools \
  ros-melodic-rtabmap-ros
RUN catkin config --extend /opt/ros/melodic && catkin build --no-status

# Automatically source the workspace when starting a bash session
RUN echo "source /catkin_ws/devel/setup.bash" >> /etc/bash.bashrc

# Install start script
COPY ./start.sh /

CMD ["/start.sh"]
