module.exports = {
    apps : [{
        name: "websocket-server",
        script: "./server.js",
        cwd: "/root/web", // カレントディレクトリを明示的に指定
        time: true,
        autorestart: false, // デバッグのため自動再起動はオフ
        // merge_logs: true, // pm2-runtime が自動でマージするため不要な場合が多い
        env: {
        NODE_ENV: "production", // 必要であれば環境変数を設定
    }
    }, {
        name: "gnss-bridge",
        script: "python3", // インタプリタとしてPythonを指定
        args: "-m gnss_bridge.gnss_bridge", // Pythonモジュールを実行するための引数
        cwd: "/root/ros2_ws", // カレントディレクトリを明示的に指定
        interpreter: "bash", // ROS2環境設定のためにbashを介して実行
        interpreter_args: "-c 'source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && exec python3 -m gnss_bridge.gnss_bridge'", // 環境設定とスクリプト実行
        time: true,
        autorestart: false, // デバッグのため自動再起動はオフ
        // merge_logs: true,
    }]
};