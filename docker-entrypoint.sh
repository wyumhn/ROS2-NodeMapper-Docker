#!/bin/bash
set -e

# ROS2 環境セットアップ
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

# pm2 プロセスに登録
cd /root/web
pm2 start server.js --name websocket-server

# ROS2ノード
cd /root/ros2_ws
pm2 start "python3 -m gnss_bridge.gnss_bridge" --name gnss-bridge

# PM2をフォアグラウンドで実行し、コンテナのメインプロセスとする
# これにより、上記で登録したプロセスが管理され、コンテナが終了するまでPM2が動作し続ける
pm2-runtime --json