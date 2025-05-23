FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    curl \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-websockets \
    python3-rosdep \
    nodejs \
    npm \
    && rm -rf /var/lib/apt/lists/*

RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update

WORKDIR /root/ros2_ws
COPY ./src ./src

RUN pip install -r src/gnss_bridge/requirements.txt
RUN rosdep install --from-paths src --ignore-src -r -y

RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select gnss_bridge

WORKDIR /root/web
RUN git clone https://github.com/wyumhn/Auto_Cosmos_mapper .
RUN cd server && npm install
RUN cd client && npm install && npm run build || true

RUN npm install -g pm2

# 起動スクリプト
COPY docker-entrypoint.sh /root/docker-entrypoint.sh
RUN chmod +x /root/docker-entrypoint.sh

ENV ROS_DOMAIN_ID=0
ENV WS_SERVER_URL=ws://localhost:3001

CMD ["/root/docker-entrypoint.sh"]