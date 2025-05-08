import rclpy
from rclpy.node import Node
import os
import asyncio
import json
import websockets

from sensor_msgs.msg import NavSatFix  # GNSS の標準メッセージ

# WebSocket送信先
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

        # asyncio イベントループを取得
        self.loop = asyncio.get_event_loop()
        self.websocket = None

    def listener_callback(self, msg):
        gps_data = {
            "id": "A",
            "lat": msg.latitude,
            "lon": msg.longitude
        }

        self.get_logger().info(f"受信: {gps_data}")

        # タスクをイベントループに登録
        self.loop.create_task(self.send_ws(gps_data))

    async def send_ws(self, data):
        try:
            if not self.websocket or self.websocket.closed:
                self.get_logger().info("WebSocket接続試行中...")
                self.websocket = await websockets.connect(WS_SERVER_URL)

            await self.websocket.send(json.dumps(data))
            self.get_logger().info(f"送信: {data}")

        except Exception as e:
            self.get_logger().warn(f"WebSocket送信失敗: {e}")


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
