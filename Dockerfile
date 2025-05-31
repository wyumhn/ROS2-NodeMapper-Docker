FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# ROS Humbleのベースイメージに含まれるapt-get updateは一度だけ実行
RUN apt-get update && apt-get install -y \
    curl \
    git \
    nano \
    python3-pip \
    python3-colcon-common-extensions \
    python3-websockets \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/* \
    && echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | tee /etc/apt/sources.list.d/zenoh.list \
    \
    && apt-get update \
    && apt-get install -y zenoh-bridge-ros2dds \
    \
    && rm -rf /var/lib/apt/lists/* # apt-getのキャッシュを削除

# Node.jsとnpmの最新LTSバージョンをインストール
RUN curl -fsSL https://deb.nodesource.com/setup_lts.x | bash - \
    && apt-get install -y nodejs \
    && rm -rf /var/lib/apt/lists/*

RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update

COPY ecosystem.config.js /root/ecosystem.config.js
COPY config.json5 /root/config.json5

WORKDIR /root/ros2_ws
COPY ./src ./src

RUN pip install -r src/gnss_bridge/requirements.txt
RUN rosdep install --from-paths src --ignore-src -r -y

RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select gnss_bridge

WORKDIR /root/web
RUN git clone https://github.com/wyumhn/Auto_Cosmos_mapper .
COPY .env.server .env
RUN npm install

RUN npm install -g pm2

# 起動スクリプト
COPY docker-entrypoint.sh /root/docker-entrypoint.sh
RUN chmod +x /root/docker-entrypoint.sh

ENV ROS_DOMAIN_ID=0
ENV WS_SERVER_URL=ws://localhost:3001

CMD ["/root/docker-entrypoint.sh"]