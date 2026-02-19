FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y \
    git \
    wget \
    curl \
    vim \
    nano \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-humble-rqt-robot-steering \
    libboost-all-dev \
    libyaml-cpp-dev \
    libsdl2-dev \
    libsdl2-image-dev \
    libsdl2-mixer-dev \
    libsdl2-ttf-dev \
    liblua5.1-0-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/flatland_ws/src

RUN git clone -b ros2-humble https://github.com/avidbots/flatland.git

WORKDIR /root/flatland_ws

RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi && \
    rosdep update --rosdistro=humble

RUN rosdep install --from-paths src --ignore-src -r -y || echo "Some dependencies may be missing"

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/flatland_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/flatland_ws

CMD ["/bin/bash"]