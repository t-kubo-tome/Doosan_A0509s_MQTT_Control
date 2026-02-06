"""Headlessモード (GUIなし) でのMQTTコマンド制御"""

import datetime
import json
import logging
import logging.handlers
import multiprocessing
import os
import queue
import signal
import time
from pathlib import Path
from typing import Optional

from dotenv import load_dotenv
from paho.mqtt import client as mqtt

from .config import ROBOT_MODEL, ROBOT_VENDOR
from .doosan_mqtt_control import ProcessManager
from .log import MicrosecondFormatter


class HeadlessLoop:
    """Headlessモード (GUIなし) でのMQTTコマンド制御ループ"""

    # MQTTによる指令がサポートされるコマンド
    SUPPORTED_COMMANDS = {
        "enable": {"params": [], "description": "アームを移動させるための電源をONにする"},
        "disable": {"params": [], "description": "アームを移動させるための電源をOFFにする"},
        "tidy_pose": {"params": [], "description": "ロボットを待機姿勢に移動する"},
        "release_hand": {"params": [], "description": "ハンドを最大まで開く"},
        "start_mqtt_control": {"params": [], "description": "MQTTでのリアルタイム制御を開始する"},
        "stop_mqtt_control": {"params": [], "description": "MQTTでのリアルタイム制御を停止する"},
        "change_log_file": {"params": [], "description": "ログ出力ディレクトリを現在時刻のディレクトリに切り替える"},
        "shutdown": {"params": [], "description": "ロボット制御プログラムを終了する"},
        "jog_joint": {
            "params": ["joint", "direction"],
            "description": "関節角度制御によるジョグ。jointは関節の順番(0-5)、directionは角度の移動量(deg)"
        },
        "jog_tcp": {
            "params": ["axis", "direction"],
            "description": "TCP座標系でのジョグ。axisは軸(0:X,1:Y,2:Z,3:RX,4:RY,5:RZ)、directionは移動量(mm)"
        },
        "get_command_list": {
            "params": [],
            "description": "サポートされているコマンド一覧を取得する"
        },
        "get_joint_names": {
            "params": [],
            "description": "ジョイントの数と名前を取得する"
        },
        # 起動時に自動的に実行するため公開しない
        # "connect_robot": {"params": [], "description": "ロボット接続"},
        # "connect_mqtt": {"params": [], "description": "MQTT接続"},
        # 本ロボットでは使用しないので公開しない
        # "clear_error": {"params": [], "description": "エラークリア"},
        # "demo_put_down_box": {"params": [], "description": "デモ実行"},
        # "line_cut": {"params": [], "description": "カッター移動"},
        # "tool_change": {"params": ["tool_id"], "description": "ツール交換"},
        # "set_area_enabled": {"params": ["enabled"], "description": "エリア設定"},
    }

    def __init__(self, **kwargs):
        self.running = True
        self.pm: Optional[ProcessManager] = None
        self.logger: Optional[logging.Logger] = None
        self.listener: Optional[logging.handlers.QueueListener] = None
        self.logging_dir: Optional[str] = None
        self.response_client: Optional[mqtt.Client] = None
        self.robot_uuid: Optional[str] = None
        self.mqtt_server: Optional[str] = None

    def _setup_response_mqtt(self) -> None:
        """MQTTレスポンス用クライアントを初期化"""
        load_dotenv(Path(__file__).parent / ".env")
        self.robot_uuid = os.getenv("ROBOT_UUID", "ur-real")
        self.mqtt_server = os.getenv("MQTT_SERVER", "localhost")
        self.response_client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        self.response_client.connect(self.mqtt_server, 1883, 60)
        self.response_client.loop_start()

    def _signal_handler(self, signum, frame):
        """Graceful shutdownのためのシグナルハンドラ"""
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

        # MQTTレスポンス用クライアントの初期化
        self._setup_response_mqtt()
        self.logger.info(f"Response MQTT client connected to {self.mqtt_server}")

    def _execute_command(self, cmd: dict) -> None:
        """コマンドを実行"""
        command_name = cmd.get("command")

        # 公開コマンドかチェック
        if command_name not in self.SUPPORTED_COMMANDS:
            self.logger.warning(f"Unknown command: {command_name}")
            return

        self.logger.info(f"Executing command: {command_name}")

        # パラメータを取得
        params = cmd.get("params", {})

        try:
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
                tool_id = params.get("tool_id")
                if tool_id is None:
                    self.logger.error("tool_change: missing params.tool_id")
                    return
                self._cmd_tool_change(int(tool_id))
            elif command_name == "set_area_enabled":
                enabled = params.get("enabled")
                if enabled is None:
                    self.logger.error("set_area_enabled: missing params.enabled")
                    return
                self._cmd_set_area_enabled(bool(enabled))
            elif command_name == "jog_joint":
                joint = params.get("joint")
                direction = params.get("direction")
                if joint is None or direction is None:
                    self.logger.error("jog_joint: missing params.joint or params.direction")
                    return
                self._cmd_jog_joint(int(joint), float(direction))
            elif command_name == "jog_tcp":
                axis = params.get("axis")
                direction = params.get("direction")
                if axis is None or direction is None:
                    self.logger.error("jog_tcp: missing params.axis or params.direction")
                    return
                self._cmd_jog_tcp(int(axis), float(direction))
            elif command_name == "change_log_file":
                self._cmd_change_log_file()
            elif command_name == "shutdown":
                self._cmd_shutdown()
            elif command_name == "get_command_list":
                self._cmd_get_command_list()
            elif command_name == "get_joint_names":
                self._cmd_get_joint_names()

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

    def _cmd_get_command_list(self):
        """サポートされているコマンド一覧をMQTTで送信"""
        if self.response_client is None:
            self.logger.warning("Response MQTT client not initialized")
            return
        response = {
            "devId": self.robot_uuid,
            "command": "get_command_list",
            "timestamp": time.time(),
            "vendor": ROBOT_VENDOR,
            "model": ROBOT_MODEL,
            "supported_commands": self.SUPPORTED_COMMANDS
        }
        topic = f"dev/{self.robot_uuid}/response"
        self.response_client.publish(topic, json.dumps(response, ensure_ascii=False))
        self.logger.info(f"Sent command list to {topic}")

    def _cmd_get_joint_names(self):
        """ジョイントの数と名前をMQTTで送信"""
        if self.response_client is None:
            self.logger.warning("Response MQTT client not initialized")
            return
        joint_names = ["J1", "J2", "J3", "J4", "J5", "J6"]
        response = {
            "devId": self.robot_uuid,
            "command": "get_joint_names",
            "timestamp": time.time(),
            "vendor": ROBOT_VENDOR,
            "model": ROBOT_MODEL,
            "joint_names": joint_names
        }
        topic = f"dev/{self.robot_uuid}/response"
        self.response_client.publish(topic, json.dumps(response, ensure_ascii=False))
        self.logger.info(f"Sent joint names to {topic}")

    def _cleanup(self) -> None:
        """クリーンアップ処理"""
        if self.logger:
            self.logger.info("Cleaning up...")
        if self.pm is not None:
            self.pm.stop_all_processes()
        print("All subprocesses stopped.")
        if self.response_client is not None:
            self.response_client.loop_stop()
            self.response_client.disconnect()
        print("Response MQTT client disconnected.")
        if self.listener is not None:
            self.listener.stop()
        print("Logging listener stopped.")
        logging.shutdown()
        print("Logging shutdown complete.")

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
