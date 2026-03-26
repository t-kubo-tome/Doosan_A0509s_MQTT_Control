# Doosanの状態をモニタリングする
import traceback

from common.monitor_base import MonitorBase


class Doosan_MON(MonitorBase):
    def _on_init(self) -> None:
        pass

    def format_error(self, e: Exception) -> str:
        # ロボット固有の処理を含む
        s = "Error trace: " + "\n" + traceback.format_exc()
        return s

    def connect_robot(self) -> None:
        pass

    def find_and_setup_hand(self, tool_id: int) -> None:
        pass

    def reconnect_robot(self) -> None:
        pass

    def disconnect_robot(self) -> None:
        pass

    def reconnect_after_timeout(self, e: Exception) -> bool:
        pass
