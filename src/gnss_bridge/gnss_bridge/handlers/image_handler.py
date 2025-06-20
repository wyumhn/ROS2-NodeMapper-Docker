import base64
from gnss_bridge.data_handler import DataHandler
from sensor_msgs.msg import Image
# 画像圧縮のためにPillow(PIL), NumPy, ioをインポート
from PIL import Image as PILImage
import numpy as np
import io
import math

class ImageHandler(DataHandler):
    """
    sensor_msgs/msg/Image メッセージを処理し、
    画像データをBase64エンコードされた文字列に変換するハンドラ
    容量が大きい場合は自動的に圧縮を行う
    """
    def process(self, msg: Image) -> dict:
        """
        Imageメッセージから画像データを抽出し、メタデータと共に
        Base64エンコードされた辞書として返す
        """
        # 圧縮後の画像サイズ（目標）
        TARGET_SIZE = (640, 480)

        try:

            numpy_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            pil_image = PILImage.fromarray(numpy_image)

            original_area = pil_image.width * pil_image.height
            scale_ratio = math.sqrt(TARGET_SIZE / original_area)

            new_width = int(pil_image.width * scale_ratio)
            new_height = int(pil_image.height * scale_ratio)

            # 4. 計算した新しいサイズにリサイズ
            resized_image = pil_image.resize((new_width, new_height), PILImage.Resampling.LANCZOS)

            buffer = io.BytesIO()
            pil_image.save(buffer, format="JPEG", quality=85)
            compressed_data = buffer.getvalue()

            image_data_base64 = base64.b64encode(compressed_data).decode('utf-8')

            return {
                "height": pil_image.height,
                "width": pil_image.width,
                "encoding": "jpeg",
                "step": pil_image.width * 3,
                "data": image_data_base64
            }

        except Exception as e:
            # 圧縮中にエラーが発生した場合は、念のため元のデータをそのまま送る
            pass

        # --- 通常処理（圧縮しない場合） ---

        image_data_base64 = base64.b64encode(msg.data).decode('utf-8')
        return {
            "height": msg.height,
            "width": msg.width,
            "encoding": msg.encoding,
            "step": msg.step,
            "data": image_data_base64
        }
