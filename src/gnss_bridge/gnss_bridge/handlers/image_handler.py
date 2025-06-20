import base64
from gnss_bridge.data_handler import DataHandler
from sensor_msgs.msg import Image
# 画像圧縮のためにPillow(PIL), NumPy, ioをインポート
from PIL import Image as PILImage
import cv2
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
        print(f"画像圧縮処理を開始:")
        # 圧縮後の画像サイズ（目標）
        TARGET_SIZE = 640 * 480

        try:
            if msg.encoding == 'bayer_gbrg8':
                # 1a. Bayerパターンのデータを1チャンネルのグレースケール画像として読み込む
                raw_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
                numpy_image = cv2.cvtColor(raw_image, cv2.COLOR_BAYER_GB2BGR)
                numpy_image = numpy_image[:, :, ::-1] # BGR -> RGB
            elif msg.encoding in ['rgb8', 'bgr8']:
                # 3チャンネルのカラー画像
                numpy_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                # BGR形式の場合は、Pillowが扱えるRGB形式にチャンネルを入れ替える
                if msg.encoding == 'bgr8':
                    numpy_image = numpy_image[:, :, ::-1] # BGR -> RGB
            elif msg.encoding == 'mono8':
                # 8-bitのグレースケール画像
                numpy_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            else:
                # サポート外のエンコーディングの場合は警告を出して処理を中断
                print(f"サポート外の画像エンコーディングです: {msg.encoding}")
                return None

            print(f"画像をnumpy行列に変換:")
            pil_image = PILImage.fromarray(numpy_image)
            print(f"画像をPIL imageに変換:")

            original_area = pil_image.width * pil_image.height

            if original_area == 0 or original_area <= TARGET_SIZE:
                scale_ratio = 1.0
            else:
                scale_ratio = math.sqrt(TARGET_SIZE / original_area)

            print(f"画像のリサイズ後の縦横比を計算: {scale_ratio}")

            new_width = int(pil_image.width * scale_ratio)
            new_height = int(pil_image.height * scale_ratio)
            print(f"画像を新しい縦横比に変更: {new_height} * {new_width}")

            # 4. 計算した新しいサイズにリサイズ
            resized_image = pil_image.resize((new_width, new_height), PILImage.Resampling.LANCZOS)
            print(f"画像をリサイズ:")

            if resized_image.mode != 'RGB':
                resized_image = resized_image.convert('RGB')

            buffer = io.BytesIO()
            resized_image.save(buffer, format="JPEG", quality=85)
            print(f"画像を JPEG 形式で保存:")
            compressed_data = buffer.getvalue()
            print(f"画像の圧縮を完了:")

            image_data_base64 = base64.b64encode(compressed_data).decode('utf-8')

            return {
                "height": resized_image.height,
                "width": resized_image.width,
                "encoding": "jpeg",
                "step": resized_image.width * 3,
                "data": image_data_base64
            }

        except Exception as e:
            # 圧縮中にエラーが発生した場合は、念のため元のデータをそのまま送る
            print(f"画像の圧縮に失敗しました: {e}")
            return None

        # --- 通常処理（圧縮しない場合） ---

        #image_data_base64 = base64.b64encode(msg.data).decode('utf-8')
        #return {
        #    "height": msg.height,
        #    "width": msg.width,
        #    "encoding": msg.encoding,
        #    "step": msg.step,
        #    "data": image_data_base64
        #}
