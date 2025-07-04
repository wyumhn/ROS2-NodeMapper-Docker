import math
from gnss_bridge.data_handler import DataHandler
from sensor_msgs.msg import Imu

class ImuHandler(DataHandler):
    """
    sensor_msgs/Imu メッセージを処理するハンドラ
    """
    def process(self, msg: Imu) -> dict:
        """
        Imuメッセージから傾き(オイラー角)と線形加速度を抽出し、
        指定された形式の辞書を返す
        """
        # --- 傾き（オイラー角）の計算 ---
        # クォータニオンの各成分を取得
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        # オイラー角をラジアンで計算
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

        # --- 線形加速度を大きさと向きに変換 ---
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # 1. 大きさ
        accel_magnitude = math.sqrt(ax**2 + ay**2 + az**2)

        # 2. 向き（方位角と仰角）

        xy_projection = math.hypot(ax, ay)
        accel_azimuth_rad = math.atan2(ay, ax)
        accel_elevation_rad = math.atan2(az, xy_projection)


        # --- 戻り値の作成 ---
        return {
            # 傾き（オイラー角）を度数法で出力
            "euler_angles": {
                "roll": math.degrees(roll_x),
                "pitch": math.degrees(pitch_y),
                "yaw": math.degrees(yaw_z)
            },
            # 線形加速度をそのまま出力
            "acceleration_spherical": {
                "magnitude": accel_magnitude,
                "azimuth": math.degrees(accel_azimuth_rad), # ヨー (z) に相当
                "elevation": math.degrees(accel_elevation_rad) # ピッチ (y) に相当
            }
        }