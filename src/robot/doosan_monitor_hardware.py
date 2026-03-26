# Doosanの状態をモニタリングする
import traceback

from common.monitor_hardware_interface import MonitorHardwareInterface


class DoosanMonitorHardware(MonitorHardwareInterface):
    def on_init(self) -> None:
        """追加の初期化処理。"""
        pass
    
    def on_del(self) -> None:
        """追加の終了処理。"""
        pass

    def format_error(self, e: Exception) -> str:
        """例外をフォーマットする。"""
        s = "Error trace: " + "\n" + traceback.format_exc()
        return s

    def start(self) -> bool:
        """ロボットに接続する。"""
        return True

    def stop(self) -> bool:
        """ロボットの接続を停止する。"""
        return True
