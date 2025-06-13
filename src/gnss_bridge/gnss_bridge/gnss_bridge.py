import rclpy
from rclpy.node import Node
import os
import asyncio
import json
import websockets
import threading
import importlib
import functools
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import NavSatFix

WS_SERVER_URL = os.environ.get('WS_SERVER_URL', 'ws://localhost:3001')


class GNSSBridge(Node):
    """
    ROSトピックを設定より動的に作成して、購読したのち
    WebSocket経由でデータを送信する
    """
    def __init__(self):
        super().__init__('gnss_bridge')
        self.get_logger().info("GNSSBridgeノード: 初期化を開始します")

        # WebSocket関連のセットアップ
        self.ws_server_url = os.environ.get('WS_SERVER_URL', 'ws://localhost:3001')
        self.queue = asyncio.Queue()
        self.loop = asyncio.new_event_loop()
        self.ws_thread = threading.Thread(target=self.loop_runner, daemon=True)
        self.ws_thread.start()

        # サブスクライバを動的に作成
        self.create_subscriptions_from_config()

        self.get_logger().info("GNSSBridgeノードが起動しました。WebSocketサーバー: {self.ws_server_url}")
        self.get_logger().info("GNSSBridgeノード: 初期化を終了します")


    # *--------------------------------------------------------
    #   サブスクライバの作成・データ受信
    # *--------------------------------------------------------


    def create_subscriptions_from_config(self):

        package_share_directory = get_package_share_directory('gnss_bridge')
        config_path = os.path.join(package_share_directory, 'config', 'topics.json')

        try:
            with open(config_path, 'r') as f:
                config_data = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError) as e:
            self.get_logger().error(f"設定ファイル '{config_path}' の読み込みに失敗しました: {e}")
            return

        for topic_config in config_data.get('topics', []):
            try:
                # メッセージ型とハンドラクラスをにインポート
                msg_type = self._import_type(topic_config['type'])
                handler_class = self._import_handler(
                    topic_config['handler_module'], topic_config['handler_class']
                )

                # ハンドラインスタンスを作成
                handler_instance = handler_class(topic_config['id'])

                self.create_subscription(
                    msg_type,
                    topic_config['name'],
                    functools.partial(self.generic_callback, handler=handler_instance, topic_name=topic_config['name']),
                    10
                )
                self.get_logger().info(f"トピック '{topic_config['name']}' のサブスクリプションを作成しました。")

            except (KeyError, ImportError, AttributeError) as e:
                self.get_logger().error(f"設定エントリ {topic_config} の処理中にエラー: {e}")

    def _import_type(self, type_str: str):
        """ 'pkg.module.Class' 形式の文字列から型をインポートする """
        module_name, class_name = type_str.rsplit('.', 1)
        module = importlib.import_module(module_name)
        return getattr(module, class_name)

    def _import_handler(self, module_name: str, class_name: str):
        """ ハンドラモジュールとクラス名からクラスをインポートする """
        full_module_name = f'gnss_bridge.handlers.{module_name}'
        module = importlib.import_module(full_module_name)
        return getattr(module, class_name)

    def generic_callback(self, msg, handler, topic_name: str):
        """すべてのサブスクリプションで共有される汎用コールバック"""
        processed_data = handler.process(msg)
        if processed_data:
            self.get_logger().debug(f"受信 ({topic_name}): {processed_data}")
            self.loop.call_soon_threadsafe(self.queue.put_nowait, processed_data)

    # *--------------------------------------------------------
    #   WebSocket接続・データ送信
    # *--------------------------------------------------------

    def loop_runner(self):
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self.ws_loop())
        finally:
            self.loop.close()

    async def ws_loop(self):
        while True:
            data = await self.queue.get()
            try:
                self.get_logger().info("WebSocket接続試行中...")
                async with websockets.connect(self.ws_server_url) as websocket:
                    self.get_logger().info(f"WebSocketサーバーに接続しました")
                    while websocket.open and rclpy.ok():
                        data = await self.queue.get()
                        await websocket.send(json.dumps(data))
                        self.get_logger().info(f"送信: {data}")
                        self.queue.task_done()
            except (websockets.exceptions.ConnectionClosed, ConnectionRefusedError) as e:
                self.get_logger().warn(f"WebSocket接続エラー: {e}。5秒後に再試行します。")
                await asyncio.sleep(5)
            except Exception as e:
                self.get_logger().error(f"WebSocketループで予期せぬエラー: {e}")
                await asyncio.sleep(5)

    def destroy_node(self):
        """ノードのクリーンアップ"""
        self.get_logger().info("ノードのシャットダウン処理を開始します。")
        # asyncioループの安全な停止
        if self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)
        # スレッドの終了を待つ
        if self.ws_thread.is_alive():
            self.ws_thread.join(timeout=2.0)
        super().destroy_node()
        self.get_logger().info("ノードのシャットダウンが完了しました。")


def main(args=None):
    rclpy.init(args=args)
    bridge_node = GNSSBridge()
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
