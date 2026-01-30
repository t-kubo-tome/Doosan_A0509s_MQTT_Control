"""Headlessモード (GUIなし) でのMQTTコマンド制御"""

import datetime
import logging
import logging.handlers
import multiprocessing
import os
import queue
import signal
import time
from typing import Optional

from .doosan_mqtt_control import ProcessManager
from .log import MicrosecondFormatter


class HeadlessLoop:
    """Headlessモード (GUIなし) でのMQTTコマンド制御ループ"""

    # サポートするコマンドの定義
    SUPPORTED_COMMANDS = {
        # 引数なしコマンド
        "connect_robot": {"params": [], "description": "ロボット接続"},
        "connect_mqtt": {"params": [], "description": "MQTT接続"},
        "enable": {"params": [], "description": "ロボット有効化"},
        "disable": {"params": [], "description": "ロボット無効化"},
        "tidy_pose": {"params": [], "description": "待機姿勢"},
        "clear_error": {"params": [], "description": "エラークリア"},
        "release_hand": {"params": [], "description": "ハンドリリース"},
        "start_mqtt_control": {"params": [], "description": "MQTT制御開始"},
        "stop_mqtt_control": {"params": [], "description": "MQTT制御停止"},
        "demo_put_down_box": {"params": [], "description": "デモ実行"},
        "line_cut": {"params": [], "description": "カッター移動"},
        "shutdown": {"params": [], "description": "シャットダウン"},
        # 引数ありコマンド
        "tool_change": {"params": ["tool_id"], "description": "ツール交換"},
        "set_area_enabled": {"params": ["enabled"], "description": "エリア設定"},
        "jog_joint": {"params": ["joint", "direction"], "description": "関節ジョグ"},
        "jog_tcp": {"params": ["axis", "direction"], "description": "TCPジョグ"},
        "change_log_file": {"params": [], "description": "ログファイル変更"},
    }

    def __init__(self, use_joint_monitor_plot: bool = False, **kwargs):
        self.use_joint_monitor_plot = use_joint_monitor_plot
        self.running = True
        self.pm: Optional[ProcessManager] = None
        self.logger: Optional[logging.Logger] = None
        self.listener: Optional[logging.handlers.QueueListener] = None
        self.logging_dir: Optional[str] = None

    def _signal_handler(self, signum, frame):
        """シグナルハンドラ: グレースフルシャットダウン"""
        if self.logger:
            self.logger.info(f"Received signal {signum}, initiating shutdown...")
        self.running = False

    def get_logging_dir(self) -> str:
        """ログディレクトリを作成して返す"""
        now = datetime.datetime.now()
        log_dir = "log"
        os.makedirs(log_dir, exist_ok=True)
        date_str = now.strftime("%Y-%m-%d")
        os.makedirs(os.path.join(log_dir, date_str), exist_ok=True)
        time_str = now.strftime("%H-%M-%S")
        logging_dir = os.path.join(log_dir, date_str, time_str)
        os.makedirs(logging_dir, exist_ok=True)
        return logging_dir

    def setup_logging(self, log_queue: multiprocessing.Queue,
                      logging_dir: str) -> None:
        """ロギングシステムを設定"""
        handlers = [
            logging.FileHandler(os.path.join(logging_dir, "log.txt")),
            logging.StreamHandler(),
        ]
        formatter = MicrosecondFormatter(
            "[%(asctime)s][%(name)s][%(levelname)s] %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S.%f",
        )
        for handler in handlers:
            handler.setFormatter(formatter)
        self.listener = logging.handlers.QueueListener(log_queue, *handlers)
        self.listener.start()

    def setup_logger(self, log_queue: multiprocessing.Queue) -> None:
        """Headlessプロセス用ロガーを設定"""
        self.logger = logging.getLogger("Headless")
        handler = logging.handlers.QueueHandler(log_queue)
        self.logger.addHandler(handler)
        self.logger.setLevel(logging.INFO)

    def _init(self) -> None:
        """初期化処理"""
        self.pm = ProcessManager(use_command_queue=True)
        self.logging_dir = self.get_logging_dir()
        self.setup_logging(self.pm.log_queue, self.logging_dir)
        self.setup_logger(self.pm.log_queue)
        self.logger.info("Headless mode started")
        self.logger.info(f"Logging to: {self.logging_dir}")

        # シグナルハンドラの設定
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _execute_command(self, cmd: dict) -> None:
        """コマンドを実行"""
        command_name = cmd.get("command")

        if command_name not in self.SUPPORTED_COMMANDS:
            self.logger.warning(f"Unknown command: {command_name}")
            return

        self.logger.info(f"Executing command: {command_name}")

        try:
            # コマンドのディスパッチ
            if command_name == "connect_robot":
                self._cmd_connect_robot()
            elif command_name == "connect_mqtt":
                self._cmd_connect_mqtt()
            elif command_name == "enable":
                self._cmd_enable()
            elif command_name == "disable":
                self._cmd_disable()
            elif command_name == "tidy_pose":
                self._cmd_tidy_pose()
            elif command_name == "clear_error":
                self._cmd_clear_error()
            elif command_name == "release_hand":
                self._cmd_release_hand()
            elif command_name == "start_mqtt_control":
                self._cmd_start_mqtt_control()
            elif command_name == "stop_mqtt_control":
                self._cmd_stop_mqtt_control()
            elif command_name == "demo_put_down_box":
                self._cmd_demo_put_down_box()
            elif command_name == "line_cut":
                self._cmd_line_cut()
            elif command_name == "tool_change":
                tool_id = cmd.get("tool_id")
                if tool_id is None:
                    self.logger.error("tool_change: missing tool_id")
                    return
                self._cmd_tool_change(int(tool_id))
            elif command_name == "set_area_enabled":
                enabled = cmd.get("enabled")
                if enabled is None:
                    self.logger.error("set_area_enabled: missing enabled")
                    return
                self._cmd_set_area_enabled(bool(enabled))
            elif command_name == "jog_joint":
                joint = cmd.get("joint")
                direction = cmd.get("direction")
                if joint is None or direction is None:
                    self.logger.error("jog_joint: missing joint or direction")
                    return
                self._cmd_jog_joint(int(joint), float(direction))
            elif command_name == "jog_tcp":
                axis = cmd.get("axis")
                direction = cmd.get("direction")
                if axis is None or direction is None:
                    self.logger.error("jog_tcp: missing axis or direction")
                    return
                self._cmd_jog_tcp(int(axis), float(direction))
            elif command_name == "change_log_file":
                self._cmd_change_log_file()
            elif command_name == "shutdown":
                self._cmd_shutdown()

            self.logger.info(f"Command completed: {command_name}")

        except Exception as e:
            self.logger.error(f"Command failed: {command_name}, error: {e}")

    # コマンド実装メソッド群
    def _cmd_connect_robot(self):
        if self.pm.state_control and self.pm.state_monitor:
            self.logger.warning("Robot already connected")
            return
        self.pm.startControl(logging_dir=self.logging_dir)
        self.pm.startMonitor(logging_dir=self.logging_dir)
        if self.use_joint_monitor_plot:
            self.pm.startMonitorGUI()

    def _cmd_connect_mqtt(self):
        if self.pm.state_recv_mqtt:
            self.logger.warning("MQTT already connected")
            return
        self.pm.startRecvMQTT()

    def _cmd_enable(self):
        if not self.pm.state_control:
            self.logger.warning("Robot not connected")
            return
        self.pm.enable()

    def _cmd_disable(self):
        if not self.pm.state_control:
            self.logger.warning("Robot not connected")
            return
        self.pm.disable()

    def _cmd_tidy_pose(self):
        if not self.pm.state_control:
            self.logger.warning("Robot not connected")
            return
        self.pm.tidy_pose()

    def _cmd_clear_error(self):
        if not self.pm.state_control:
            self.logger.warning("Robot not connected")
            return
        self.pm.clear_error()

    def _cmd_release_hand(self):
        if not self.pm.state_control:
            self.logger.warning("Robot not connected")
            return
        self.pm.release_hand()

    def _cmd_start_mqtt_control(self):
        if not (self.pm.state_control and self.pm.state_monitor and
                self.pm.state_recv_mqtt):
            self.logger.warning("Not ready for MQTT control")
            return
        self.pm.start_mqtt_control()

    def _cmd_stop_mqtt_control(self):
        if not (self.pm.state_control and self.pm.state_monitor and
                self.pm.state_recv_mqtt):
            self.logger.warning("Not in MQTT control mode")
            return
        self.pm.stop_mqtt_control()

    def _cmd_demo_put_down_box(self):
        if not self.pm.state_control:
            self.logger.warning("Robot not connected")
            return
        self.pm.demo_put_down_box()

    def _cmd_line_cut(self):
        if not self.pm.state_control:
            self.logger.warning("Robot not connected")
            return
        self.pm.line_cut()

    def _cmd_tool_change(self, tool_id: int):
        if not self.pm.state_control:
            self.logger.warning("Robot not connected")
            return
        self.pm.tool_change(tool_id)

    def _cmd_set_area_enabled(self, enabled: bool):
        if not self.pm.state_control:
            self.logger.warning("Robot not connected")
            return
        self.pm.set_area_enabled(enabled)

    def _cmd_jog_joint(self, joint: int, direction: float):
        if not self.pm.state_control:
            self.logger.warning("Robot not connected")
            return
        self.pm.jog_joint(joint, direction)

    def _cmd_jog_tcp(self, axis: int, direction: float):
        if not self.pm.state_control:
            self.logger.warning("Robot not connected")
            return
        self.pm.jog_tcp(axis, direction)

    def _cmd_change_log_file(self):
        if self.listener is not None:
            self.listener.stop()
        self.logging_dir = self.get_logging_dir()
        self.pm.change_log_file(self.logging_dir)
        self.logger.info(f"Changed log directory to {self.logging_dir}")
        # サブプロセスの完了を待つ
        while self.pm.ar[33] != 0 or self.pm.ar[34] != 0:
            time.sleep(0.1)
        self.setup_logging(self.pm.log_queue, self.logging_dir)

    def _cmd_shutdown(self):
        self.logger.info("Shutdown command received")
        self.running = False

    def _cleanup(self) -> None:
        """クリーンアップ処理"""
        if self.logger:
            self.logger.info("Cleaning up...")
        if self.pm is not None:
            self.pm.stop_all_processes()
        if self.listener is not None:
            self.listener.stop()
        logging.shutdown()

    def mainloop(self) -> None:
        """メインループ"""
        self._init()

        # 起動時に自動接続
        self.logger.info("Auto-connecting Robot...")
        self._cmd_connect_robot()
        self.logger.info("Auto-connecting MQTT...")
        self._cmd_connect_mqtt()

        try:
            while self.running:
                # コマンドキューからコマンドを取得
                try:
                    cmd = self.pm.command_queue.get(block=True, timeout=1.0)
                    self._execute_command(cmd)
                except queue.Empty:
                    # タイムアウト: ループを継続
                    pass
        finally:
            self._cleanup()
