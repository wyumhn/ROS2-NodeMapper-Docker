FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-websockets \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update

WORKDIR /root/ros2_ws
COPY ./src ./src

RUN pip install -r src/gnss_bridge/requirements.txt
RUN rosdep install --from-paths src --ignore-src -r -y

RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select gnss_bridge

ENV ROS_DOMAIN_ID=0
ENV WS_SERVER_URL=ws://localhost:3000

CMD bash -c "source /opt/ros/humble/setup.bash && \
            source /root/ros2_ws/install/setup.bash && \
            ros2 run gnss_bridge gnss_bridge"
