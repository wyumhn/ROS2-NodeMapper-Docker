from ros_ws_bridge_package.data_handler import DataHandler
from sensor_msgs.msg import Imu

class ImuHandler(DataHandler):
    """
    sensor_msgs/Imu メッセージを処理するハンドラ。
    """
    def process(self, msg: Imu) -> dict:
        """
        Imuメッセージから角速度と線形加速度を抽出し、指定された形式の辞書を返す。
        """
        return {
            "id": self.id,
            "angular_velocity": {
                "x": msg.angular_velocity.x,
                "y": msg.angular_velocity.y,
                "z": msg.angular_velocity.z
            },
            "linear_acceleration": {
                "x": msg.linear_acceleration.x,
                "y": msg.linear_acceleration.y,
                "z": msg.linear_acceleration.z
            }
        }