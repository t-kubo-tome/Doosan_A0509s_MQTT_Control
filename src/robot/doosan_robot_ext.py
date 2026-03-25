import logging
import time
import threading

from .doosan_robot import ROBOT_STATE, DoosanRobot


class DoosanRobotExt(DoosanRobot):
    """Pybindで作成したDoosanRobotを拡張するクラス.ログの取得機能を追加."""
    def __init__(
        self,
        ip: str,
        log: str,
        fPeriod: float,
        logger: logging.Logger | None = None,
        log_t_intv: float | None = None,
    ) -> None:
        self._log_type = log
        if self._log_type == "queue":
            if logger is None:
                self._logger = logging.getLogger(__name__)
            else:
                self._logger = logger
            # ログの優先度は低いため周期を長くすることを推奨
            if log_t_intv is None:
                self._log_t_intv = fPeriod * 2
            else:
                self._log_t_intv = log_t_intv
            self._log_thread = threading.Thread(
                target=self._log_loop_in_python)
            self._log_thread.start()
        super().__init__(ip, self._log_type, fPeriod)

    def stop_log_if_exists(self) -> None:
        """ログの種類がキュー (Pythonロガーを使用) であればログスレッドを停止します。"""
        if self._log_type == "queue":
            self._log_running = False
            self._log_thread.join()

    def _log_loop_in_python(self) -> None:
        """ログの種類がキュー (Pythonロガーを使用) でのログのループ処理を行います。"""
        # 停止されるまでログを処理し続ける
        self._log_running = True
        while self._log_running:
            now = time.time()

            # DoosanRobotのメソッド。例外処理不要
            log_block = self.pop_log_queue()

            for log in log_block:
                # log is a tuple: (timestamp, level, message)
                timestamp, level, message = log                
                # ログレコードを手動で作成してタイムスタンプを反映
                log_record = logging.LogRecord(
                    name=self._logger.name,
                    level=getattr(logging, level, logging.INFO),
                    pathname="",
                    lineno=0,
                    msg=message,
                    args=(),
                    exc_info=None
                )
                # タイムスタンプを設定（Unix timestamp）
                log_record.created = timestamp
                log_record.msecs = (timestamp - int(timestamp)) * 1000
                # ログレコードをハンドラーに直接渡す
                if self._logger.isEnabledFor(log_record.levelno):
                    self._logger.handle(log_record)

            t_elapsed = time.time() - now
            t_wait = self._log_t_intv - t_elapsed
            if t_wait > 0:
                time.sleep(t_wait)


__all__ = ["ROBOT_STATE", "DoosanRobotExt"]
