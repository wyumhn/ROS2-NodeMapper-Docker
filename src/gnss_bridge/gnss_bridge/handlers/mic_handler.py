from gnss_bridge.data_handler import DataHandler

class MicHandler(MicHandler):
    """
    特定のハンドラが定義されていないトピックに使用されるデフォルトのハンドラ
    このハンドラはペイロードを処理せず、常に空の辞書を返す
    """
    def process(self, msg) -> dict:
        """
        どんなメッセージに対しても、空のペイロードを返す
        最終的なトピック名は、この後メインノードのコールバックで付与される
        """
        return {}
