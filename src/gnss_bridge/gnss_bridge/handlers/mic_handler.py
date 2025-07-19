from gnss_bridge.data_handler import DataHandler
from std_msgs.msg import Float32MultiArray

class MicHandler(DataHandler):
    def process(self, msg: Float32MultiArray) -> dict:
        if len(msg.data) >= 8:
            return {
                "data1": msg.data[0],
                "data2": msg.data[1],
                "data3": msg.data[2],
                "data4": msg.data[3],
                "data5": msg.data[4],
                "data6": msg.data[5],
                "data7": msg.data[6],
                "data8": msg.data[7]
            }
        else:
            print("Warning: Float32MultiArray.data does not contain at least 8 elements.")
            return {}

