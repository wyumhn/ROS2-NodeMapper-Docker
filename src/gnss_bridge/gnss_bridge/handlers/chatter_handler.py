from gnss_bridge.data_handler import DataHandler
from sensor_msgs.msg import NavSatFix

class ChatterHandler(DataHandler):
    """
    sensor_msgs/NavSatFix メッセージを処理する
    """
    def process(self, msg: NavSatFix) -> dict:
        """
        NavSatFixメッセージから緯度と経度を抽出し、指定された形式の辞書を返す
        """
        return {
            "data": msg.data,
        }