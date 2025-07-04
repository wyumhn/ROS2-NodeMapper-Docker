import math
from gnss_bridge.data_handler import DataHandler
from sensor_msgs.msg import Imu

class ImuHandler(DataHandler):
    """
    sensor_msgs/Imu メッセージを処理するハンドラ
    """
    def process(self, msg: Imu) -> dict:
        """
        Imuメッセージから角速度と線形加速度を抽出し、指定された形式の辞書を返す
        """
        # クォータニオンの各成分を取得
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        # オイラー角（ロール、ピッチ、ヨー）
        # ロール (x軸)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        # ピッチ (y軸)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        # ヨー (z軸)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return {
            "euler_angles": {
                "roll": roll_x,
                "pitch": pitch_y,
                "yaw": yaw_z
            }
        }