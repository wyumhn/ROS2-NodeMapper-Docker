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
from rosidl_runtime_py.convert import message_to_ordereddict
from rclpy.serialization import serialize_message
import re
from gnss_bridge.handlers.default_handler import DefaultHandler

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
        self.subscribed_topics = set()

        self.create_subscriptions_from_config()

        self.discovery_timer = self.create_timer(5.0, self.discover_and_subscribe_topics)
        self.get_logger().info("新しいトピックの動的検出を開始しました")

        self.get_logger().info("GNSSBridgeノードが起動しました。WebSocketサーバー: {self.ws_server_url}")
        self.get_logger().info("GNSSBridgeノード: 初期化を終了します")


    def load_config(self):
        package_share_directory = get_package_share_directory('gnss_bridge')
        config_path = os.path.join(package_share_directory, 'config', 'topics.json')

        # デフォルトで無視する内部トピック
        default_ignores = ['/rosout', '/parameter_events']

        try:
            with open(config_path, 'r') as f:
                config_data = json.load(f)

            self.topic_configs = config_data.get('topics', [])
            self.ignore_list = config_data.get('ignore_list', [])

            for topic in default_ignores:
                if topic not in self.ignore_list:
                    self.ignore_list.append(topic)

            self.get_logger().info(f"設定ファイルから読み込んだ無視リスト: {self.ignore_list}")

        except (FileNotFoundError, json.JSONDecodeError):
            self.get_logger().warn(f"設定ファイル'{config_path}'が見つからないか無効です。デフォルトの無視リストを使用します。")
            self.ignore_list = default_ignores

    # *--------------------------------------------------------
    #   サブスクライバの作成・データ受信
    # *--------------------------------------------------------


    def discover_and_subscribe_topics(self):

        self.get_logger().info("--- デバッグ ---")
        current_topics = self.get_topic_names_and_types()

        for topic_name, msg_types in current_topics:
            if topic_name not in self.subscribed_topics:
                # msg_typesはリストなので最初の要素を使用
                msg_type_str = msg_types[0]
                self.get_logger().info(f"新しいトピックを検出: '{topic_name}' (型: {msg_type_str})")
                self.add_subscription_for_topic(topic_name, msg_type_str)

    def create_subscriptions_from_config(self):

        package_share_directory = get_package_share_directory('gnss_bridge')
        config_path = os.path.join(package_share_directory, 'config', 'topics.json')
        self.get_logger().info(f"パス: {config_path}")

        try:
            with open(config_path, 'r') as f:
                config_data = json.load(f)
            for topic_config in config_data.get('topics', []):
                self.add_subscription_for_topic(
                    topic_name=topic_config['name'],
                    msg_type_str=topic_config['type'],
                    handler_info=topic_config  # ハンドラ情報を渡す
                )
        except (FileNotFoundError, json.JSONDecodeError) as e:
            self.get_logger().error(f"設定ファイル '{config_path}' の読み込みに失敗しました: {e}")

    def add_subscription_for_topic(self, topic_name, msg_type_str, handler_info=None):
        # ★ 修正点2: ハードコーディングではなく、ignore_listでチェック
        if topic_name in self.ignore_list:
            self.get_logger().debug(f"トピック '{topic_name}' は無視リストに含まれているためスキップします。")
            return

        if topic_name in self.subscribed_topics:
            return


        handler_instance = None

        try:
            if handler_info and 'handler_module' in handler_info:

                handler_class = self._import_handler(handler_info['handler_module'], handler_info['handler_class'])
                self.get_logger().info(f"'{topic_name}'に設定ファイルからハンドラ'{handler_info['handler_class']}'を適用します。")
            else:

                module_name, class_name = self._handler_name_from_msg_type(msg_type_str)
                handler_class = self._import_handler(module_name, class_name)
                self.get_logger().info(f"'{topic_name}'に規約ベースのハンドラ'{class_name}'を適用します。")
            handler_instance = handler_class()
        except (ImportError, AttributeError, TypeError) as e:
            self.get_logger().error(f"'{topic_name}'のハンドラ読み込みに失敗しました！ 原因: [{type(e).__name__}] {e}")
            self.get_logger().warn(f"'{topic_name}'に対応するハンドラが見つかりません。DefaultHandlerを使用します。")
            handler_instance = DefaultHandler()

        # サブスクリプション作成
        try:
            msg_type = self._import_type(msg_type_str)

            callback_with_args = functools.partial(
                self.generic_callback,
                handler=handler_instance,
                topic_name=topic_name,
                msg_type_str=msg_type_str
            )

            self.create_subscription(
                msg_type,
                topic_name,
                callback_with_args,
                10
            )
            self.subscribed_topics.add(topic_name)
            self.get_logger().info(f"トピック'{topic_name}'のサブスクリプションを作成しました。")
        except Exception as e:
            self.get_logger().error(f"'{topic_name}'のサブスクリプション作成に失敗しました: {e}")

    def _handler_name_from_msg_type(self, msg_type_str):
        """'pkg/msg/TypeName' から ('type_name_handler', 'TypeNameHandler') を生成"""
        if '/' in msg_type_str:
            type_name = msg_type_str.split('/')[-1]
        else:
            type_name = msg_type_str
        #例: TypeName -> type_name
        module_name_snake = re.sub(r'(?<!^)(?=[A-Z])', '_', type_name).lower()
        return f"{module_name_snake}_handler", f"{type_name}Handler"

    def _import_type(self, type_str: str):
        """ 'pkg.module.Class' 形式の文字列から型をインポートする """

        py_type_str = type_str.replace('/', '.')
        parts = py_type_str.rsplit('.', 1)

        if len(parts) == 2:
            # 正常なケース: ['package.msg', 'Type']
            module_name = parts[0]
            class_name = parts[1]
            module = importlib.import_module(module_name)
            return getattr(module, class_name)
        else:
            # 異常なケース: '.' で分割できない、または予期せぬ形式
            raise ImportError(f"メッセージ型 '{type_str}' は 'package.msg.Type' の形式ではないため、インポートできません。")

    def _import_handler(self, module_name: str, class_name: str):
        """ ハンドラモジュールとクラス名からクラスをインポートする """
        full_module_name = f'gnss_bridge.handlers.{module_name}'
        module = importlib.import_module(full_module_name)
        return getattr(module, class_name)

    def generic_callback(self, msg, handler, topic_name: str, msg_type_str: str):
        """すべてのサブスクリプションで共有される汎用コールバック"""
        self.get_logger().debug(f"受信 ({topic_name}): ハンドラ名 {handler}")
        payload = handler.process(msg)
        if payload is not None:
            # ここで topic 属性を追加
            payload['topic'] = topic_name
            self.get_logger().debug(f"キューに追加 ({topic_name}): {payload}")
            self.loop.call_soon_threadsafe(self.queue.put_nowait, payload)


        #try:
        #
        #    # 容量が10KB (10 * 1024 bytes) を超えているか
        #    if msg_type_str not in ['sensor_msgs/msg/Image', 'sensor_msgs/msg/PointCloud2']:
        #        self.get_logger().info(f"'{topic_name}' の生データを送信します。")
        #
        #        # ROSメッセージ全体を辞書に変換
        #        raw_data_dict = message_to_ordereddict(msg)
        #        self.get_logger().info(f"[生データ] キューに追加 (: {raw_data_dict}")
        #        self.loop.call_soon_threadsafe(self.queue.put_nowait, raw_data_dict)
        #
        #    else:
        #        self.get_logger().info(f"'{topic_name}' は Image 型なので、生データを送信しません。（: {raw_data_dict['topic']}）")
        #
        #except Exception as e:
        #    self.get_logger().error(f"生データの変換に失敗 ({topic_name}): {e}")


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

                async with websockets.connect(self.ws_server_url) as websocket:
                    await websocket.send(json.dumps(data))
                    #self.get_logger().info(f"送信成功: {data}")

            except (websockets.exceptions.ConnectionClosed, ConnectionRefusedError, OSError) as e:
                self.get_logger().warn(f"WebSocket接続または送信に失敗しました: {e}。次のメッセージで再試行します: {data}")
            except Exception as e:
                self.get_logger().error(f"WebSocket処理中に予期せぬエラーが発生しました: {e}。: {data}")

            # キューのタスクが完了したことを通知
            self.queue.task_done()

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
