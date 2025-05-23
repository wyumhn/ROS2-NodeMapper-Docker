#!/bin/bash
set -e

# ROS2 環境セットアップ
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

# WebSocketサーバ 起動（バックグラウンド）
cd /root/web/server
node app.js &

# 必要ならクライアントも起動（ポート3001）
cd /root/web/client
npm run start &

# ROS2 ノード起動（フォアグラウンド）
python3 -m gnss_bridge.gnss_bridge