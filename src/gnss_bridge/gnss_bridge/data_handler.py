from abc import ABC, abstractmethod

class DataHandler(ABC):
    """
    ROSメッセージを処理し、WebSocketで送信するデータ形式に変換するクラス
    """
    def __init__(self, topic_id: str):
        """
        ハンドラの初期化
        """
        self.id = topic_id

    @abstractmethod
    def process(self, msg):
        """
        受信したROSメッセージを処理し、辞書形式のデータを変換する
        変換不可能な場合や送信すべきでないデータの場合、Noneを返す

        :param msg: 受信したROSメッセージ
        :return: WebSocketで送信するデータ（辞書形式）
        """
        pass