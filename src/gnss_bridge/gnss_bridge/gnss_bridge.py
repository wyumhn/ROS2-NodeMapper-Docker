import rclpy
from rclpy.node import Node
import os
import asyncio
import json
import websockets
import threading

from sensor_msgs.msg import NavSatFix

WS_SERVER_URL = os.environ.get('WS_SERVER_URL', 'ws://localhost:3001')


class GNSSBridge(Node):
    def __init__(self):
        super().__init__('gnss_bridge')
        self.get_logger().info("GNSSBridgeノード: __init__を開始します。")
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.listener_callback,
            10
        )

        self.queue = asyncio.Queue()
        self.loop = asyncio.new_event_loop()

        self.ws_thread = threading.Thread(target=self.loop_runner, daemon=True)
        self.ws_thread.start()

        self.get_logger().info("GNSSBridgeノードが起動しました。")
        self.get_logger().info("GNSSBridgeノード: __init__を終了します。")



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
    logger = rclpy.logging.get_logger('gnss_bridge_main')
    logger.info("main() 関数を開始します。")

    try:
        logger.info("rclpy.init() を呼び出します。")
        rclpy.init(args=args)
        logger.info("rclpy.init() が完了しました。")

        logger.info("GNSSBridgeインスタンスを作成します。")
        bridge = GNSSBridge()
        logger.info("GNSSBridgeインスタンスの作成が完了しました。")

        logger.info("rclpy.spin(bridge) を呼び出します。")
        rclpy.spin(bridge)
        logger.info("rclpy.spin() が終了しました。")

    except KeyboardInterrupt:
        logger.info('KeyboardInterrupt を受信しました。ノードをシャットダウンします。')
    except Exception as e:
        logger.error(f'予期せぬエラーが発生しました: {e}', exc_info=True) # exc_info=True でスタックトレースも出力
    finally:
        logger.info('ノードのクリーンアップとROS 2シャットダウンを開始します。')
        if 'bridge' in locals() and bridge.loop.is_running(): # bridge オブジェクトが存在するか確認
            logger.info('WebSocketイベントループの停止を試みます。')
            bridge.loop.call_soon_threadsafe(bridge.loop.stop)
            # スレッドが終了するまで少し待つ (任意だが推奨)
            # bridge.ws_thread.join(timeout=2) # 短いタイムアウト

        if 'bridge' in locals(): # bridge オブジェクトが存在する場合のみ
            logger.info('bridge.destroy_node() を呼び出します。')
            bridge.destroy_node() # ノードをクリーンアップ
            logger.info('bridge.destroy_node() が完了しました。')

        if rclpy.ok(): # rclpyがまだ初期化状態にあるか確認
            logger.info('rclpy.shutdown() を呼び出します。')
            rclpy.shutdown() # ROS 2コンテキストのシャットダウン
            logger.info('rclpy.shutdown() が正常に呼び出されました。')
        else:
            logger.warning('rclpyはすでにシャットダウンされているか、初期化されていません。rclpy.shutdown() はスキップします。')



if __name__ == '__main__':
    main()
