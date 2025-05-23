#!/bin/bash
set -e

# ROS2 環境セットアップ
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

# pm2 プロセスに登録
cd /root/web/server
pm2 start app.js --name websocket-server

cd /root/web/client
pm2 start "npm run start" --name web-client --interpreter bash

# ROS2ノード
cd /root/ros2_ws
pm2 start "python3 -m gnss_bridge.gnss_bridge" --name gnss-bridge --interpreter bash

# コンテナが終了しないようにフォアグラウンドで pm2 を保持
pm2 logs