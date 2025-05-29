module.exports = {
    apps : [{
        name: "websocket-server",
        script: "./server.js",
        cwd: "/root/web",
        time: true,
        autorestart: false,
        env: {
        NODE_ENV: "production",
    }
    }, {
        name: "gnss-bridge",
        script: "python3",
        args: "-m gnss_bridge.gnss_bridge",
        cwd: "/root/ros2_ws",
        interpreter: "bash",
        interpreter_args: "-c 'source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && exec python3 -m gnss_bridge.gnss_bridge'",
        time: true,
        autorestart: false,
    }, {
        name: "zenoh-bridge-ros2dds",
        script: "bash",
        args: "-c 'source /opt/ros/humble/setup.bash && exec zenoh-bridge-ros2dds --config /root/config.json5'",
        cwd: "/root/",
        time: true,
        autorestart: false,
    }]
};