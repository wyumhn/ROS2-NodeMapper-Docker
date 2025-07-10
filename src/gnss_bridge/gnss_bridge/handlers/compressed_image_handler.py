import base64
from gnss_bridge.data_handler import DataHandler
from sensor_msgs.msg import CompressedImage

from PIL import Image as PILImage
import numpy as np
import io
import math

import cv2

class CompressedImageHandler(DataHandler):
    def process(self, msg: CompressedImage) -> dict:

        TARGET_AREA = 640 * 480

        try:
            # 1. 受信した圧縮データ(msg.data)をNumPy配列に変換
            np_arr = np.frombuffer(msg.data, np.uint8)

            # 2. NumPy配列をOpenCVを使って画像にデコード（解凍）
            # cv2.IMREAD_COLORは画像をカラーとして読み込むフラグ
            cv2_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # 3. OpenCV(BGR)からPillow(RGB)で扱える形式に変換
            rgb_image = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(rgb_image)

            original_area = pil_image.width * pil_image.height

            if original_area > 0 and original_area > TARGET_AREA:
                scale_ratio = math.sqrt(TARGET_AREA / original_area)
            else:
                scale_ratio = 1.0

            new_width = int(pil_image.width * scale_ratio)
            new_height = int(pil_image.height * scale_ratio)

            resized_image = pil_image.resize((new_width, new_height), PILImage.Resampling.LANCZOS)

            buffer = io.BytesIO()
            resized_image.save(buffer, format="JPEG", quality=85)
            compressed_data = buffer.getvalue()

            image_data_base64 = base64.b64encode(compressed_data).decode('utf-8')

            return {
                "height": resized_image.height,
                "width": resized_image.width,
                "encoding": "jpeg",
                "step": resized_image.width * 3,
                "data": image_data_base64
            }

        except Exception as e:
            print(f"圧縮済み画像の処理に失敗しました: {e}")
            return None
