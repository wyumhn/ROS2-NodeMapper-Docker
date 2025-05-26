#!/bin/bash
set -e

echo "--- docker-entrypoint.sh 開始 ---"

# ROS2 環境セットアップ
echo "ROS2環境セットアップ: /opt/ros/humble/setup.bash を source します。"
source /opt/ros/humble/setup.bash
echo "ROS2環境セットアップ: /root/ros2_ws/install/setup.bash を source します。"
source /root/ros2_ws/install/setup.bash
echo "ROS2環境セットアップ完了。"

# pm2 プロセスに登録
echo "websocket-server を PM2 に登録します。"
cd /root/web
pm2 start server.js --name websocket-server --time --no-autorestart --json > /dev/null # ログはPM2が管理するので標準出力は抑制
echo "websocket-server 登録完了。"

echo "gnss-bridge を PM2 に登録します。"
cd /root/ros2_ws
pm2 start "python3 -m gnss_bridge.gnss_bridge" --name gnss-bridge --time --no-autorestart --json > /dev/null # ログはPM2が管理するので標準出力は抑制
echo "gnss-bridge 登録完了。"

echo "PM2 Runtime をフォアグラウンドで実行します。"
# PM2をフォアグラウンドで実行し、コンテナのメインプロセスとする
# このコマンドがスクリプトの最後の実効行である必要があります。
exec pm2-runtime

echo "--- docker-entrypoint.sh 終了 (通常はここには到達しません) ---"