from abc import ABC, abstractmethod

class DataHandler(ABC):
    """
    ROSメッセージを処理し、純粋なデータ部分を抽出するクラス
    """
    @abstractmethod
    def process(self, msg):
        """
        受信したROSメッセージを処理し、データを辞書形式で返す

        :param msg:
        :return:
        """
        pass
