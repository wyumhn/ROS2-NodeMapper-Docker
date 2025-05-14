import rclpy
from rclpy.node import Node
import os
import asyncio
import json
import websockets
import threading

from sensor_msgs.msg import NavSatFix

WS_SERVER_URL = os.environ.get('WS_SERVER_URL', 'ws://localhost:3000')


class GNSSBridge(Node):
    def __init__(self):
        super().__init__('gnss_bridge')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.listener_callback,
            10
        )

        self.queue = asyncio.Queue()
        self.loop = asyncio.new_event_loop()

        threading.Thread(target=self.loop_runner, daemon=True).start()

    def loop_runner(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.ws_loop())

    async def ws_loop(self):
        while True:
            data = await self.queue.get()
            try:
                self.get_logger().info("WebSocket接続試行中...")
                async with websockets.connect(WS_SERVER_URL) as websocket:
                    await websocket.send(json.dumps(data))
                    self.get_logger().info(f"送信: {data}")
            except Exception as e:
                self.get_logger().warn(f"WebSocket送信失敗: {e}")

    def listener_callback(self, msg):
        gps_data = {
            "id": "A",
            "lat": msg.latitude,
            "lon": msg.longitude
        }

        self.get_logger().info(f"受信: {gps_data}")
        self.loop.call_soon_threadsafe(self.queue.put_nowait, gps_data)


def main(args=None):
    rclpy.init(args=args)
    bridge = GNSSBridge()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
