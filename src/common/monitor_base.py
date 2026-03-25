# ロボットの状態をモニタリングする
import json
import logging
import logging.handlers
import os
import queue
import sys
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum, auto
from typing import Any, Dict, List, TextIO

import psutil
from paho.mqtt import client as mqtt

from .shared_memory import NamedSharedMemoryBase
from .utils import AngleUnitConverter


@dataclass
class MonitorConfig:
    mqtt_server: str
    mqtt_robot_state_topic: str  # "robot/<robot_uuid>" 形式
    save_state: bool
    joint_unit_internal: str
    joint_unit_external: str


class LoopResult(Enum):
    NOT_CONNECTED = auto()
    INTERRUPTED = auto()
    LOG_FILE_CHANGED = auto()


class MonitorBase(ABC):
    """ロボットのモニタリングループの基底クラス."""

    # START: 実装必須

    @abstractmethod
    def _get_config(self) -> MonitorConfig:
        pass

    @abstractmethod
    def _get_make_shared_memory(self) -> type[NamedSharedMemoryBase]:
        pass

    @abstractmethod
    def _init_other_than_config(self) -> None:
        pass

    @abstractmethod
    def format_error(self, e: Exception) -> str:
        pass

    @abstractmethod
    def connect_robot(self) -> None:
        pass

    @abstractmethod
    def find_and_setup_hand(self, tool_id: int) -> None:
        pass

    @abstractmethod
    def reconnect_robot(self) -> None:
        pass

    @abstractmethod
    def disconnect_robot(self) -> None:
        pass

    @abstractmethod
    def reconnect_after_timeout(self, e: Exception) -> bool:
        pass

    # STOP: 実装必須

    # START: オーバーライドの可能性なし

    def __init__(self) -> None:
        self.config = self._get_config()
        self._init_other_than_config()
        self._angle_unit_converter = AngleUnitConverter(
            self.config.joint_unit_internal, self.config.joint_unit_external)

    def init_realtime(self) -> None:
        os_used = sys.platform
        process = psutil.Process(os.getpid())
        if os_used == "win32":  # Windows (either 32-bit or 64-bit)
            process.nice(psutil.REALTIME_PRIORITY_CLASS)
        elif os_used == "linux":  # linux
            rt_app_priority = 80
            param = os.sched_param(rt_app_priority)
            try:
                os.sched_setscheduler(0, os.SCHED_FIFO, param)
            except OSError:
                self.logger.warning("Failed to set real-time process scheduler to %u, priority %u" % (os.SCHED_FIFO, rt_app_priority))
            else:
                self.logger.info("Process real-time priority set to: %u" % rt_app_priority)

    def on_connect(self, client, userdata, connect_flags, reason_code, properties) -> None:
        self.logger.info("MQTT connected with result code: " + str(reason_code))

    def on_disconnect(
        self,
        client,
        userdata,
        disconnect_flags,
        reason_code,
        properties,
    ) -> None:
        if reason_code != 0:
            self.logger.warning("MQTT unexpected disconnection.")

    def connect_mqtt(self, disable_mqtt: bool = False) -> None:
        if disable_mqtt:
            self.client = None
            return
        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.connect(self.config.mqtt_server, 1883, 60)
        self.client.loop_start()

    def get_tool_info(
        self, tool_infos: List[Dict[str, Any]], tool_id: int,
    ) -> Dict[str, Any]:
        return [tool_info for tool_info in tool_infos
                if tool_info["id"] == tool_id][0]

    def monitor_start(self, f: TextIO | None = None) -> LoopResult:
        last = 0
        while True:
            # ログファイル変更時
            if self.shm.change_log_file_monitor == 1:
                return LoopResult.LOG_FILE_CHANGED
            # 中断時
            if self.shm.exit_program == 1:
                return LoopResult.INTERRUPTED
            # ループが回り続けるようにタイムアウトを設定
            try:
                actual_joint_js = self.monitor_queue.get(block=True, timeout=1)
            except queue.Empty:
                continue

            now = actual_joint_js["time"]
            if last == 0:
                last = now

            if now - last > 0.3 or "tool_change" in actual_joint_js or "put_down_box" in actual_joint_js:
                if self.client is not None:
                    jss = json.dumps(actual_joint_js)
                    self.client.publish(self.config.mqtt_robot_state_topic, jss)
                    actual_joint_js["topic"] = self.config.mqtt_robot_state_topic
                self.topic_memory.write("robot", actual_joint_js)
                last = now

            # MQTT手動制御モード時のみ記録する
            # それ以外の時のエラーはstate情報は必要ないと考えたため
            if f is not None and self.shm.is_mqtt_control == 1:
                joints = actual_joint_js.get("joints")
                if joints is not None:
                    joints = self._angle_unit_converter.to_external_list(joints)
                datum = dict(
                    time=now,
                    kind="state",
                    joint=joints,
                    pose=actual_joint_js.get("poses"),
                    forces=actual_joint_js.get("forces"),
                    error=actual_joint_js.get("error", {}),
                    enabled=actual_joint_js["enabled"],
                )
                js = json.dumps(datum, ensure_ascii=False)
                f.write(js + "\n")

    def setup_logger(self, log_queue) -> None:
        self.logger = logging.getLogger("MON")
        if log_queue is not None:
            self.handler = logging.handlers.QueueHandler(log_queue)
        else:
            self.handler = logging.StreamHandler()
        self.logger.addHandler(self.handler)
        self.logger.setLevel(logging.INFO)
        self.robot_logger = logging.getLogger("MON-ROBOT")
        if log_queue is not None:
            self.robot_handler = logging.handlers.QueueHandler(log_queue)
        else:
            self.robot_handler = logging.StreamHandler()
        self.robot_logger.addHandler(self.robot_handler)
        self.robot_logger.setLevel(logging.WARNING)

    def get_logging_dir_and_change_log_file(self) -> None:
        command = self.monitor_pipe.recv()
        logging_dir = command["params"]["logging_dir"]
        self.logger.info("Change log file")
        self.logging_dir = logging_dir
        self.shm.change_log_file_monitor = 0

    def run_proc(
        self,
        topic_memory,
        slave_mode_lock,
        log_queue,
        monitor_pipe,
        monitor_queue,
        logging_dir,
        disable_mqtt: bool = False,
    ) -> None:
        self.setup_logger(log_queue)
        self.logger.info("Process started")
        self.shm = self._get_make_shared_memory()(create=False)
        self.topic_memory = topic_memory
        self.slave_mode_lock = slave_mode_lock
        self.monitor_pipe = monitor_pipe
        self.monitor_queue = monitor_queue
        self.logging_dir = logging_dir

        self.init_realtime()
        self.connect_robot()
        self.connect_mqtt(disable_mqtt=disable_mqtt)
        while True:
            try:
                if self.config.save_state:
                    with open(
                        os.path.join(self.logging_dir, "state.jsonl"), "a"
                    ) as f:
                        res = self.monitor_start(f)
                else:
                    res = self.monitor_start()
                if res == LoopResult.LOG_FILE_CHANGED:
                    self.get_logging_dir_and_change_log_file()
                elif res == LoopResult.INTERRUPTED:
                    self.disconnect_robot()
                    if self.client is not None:
                        self.client.loop_stop()
                        self.client.disconnect()
                    self.monitor_queue.close()
                    self.shm.release()
                    time.sleep(1)
                    self.logger.info("Process stopped")
                    self.handler.close()
                    self.robot_handler.close()
                    break
                elif res == LoopResult.NOT_CONNECTED:
                    self.reconnect_robot()
            except Exception as e:
                self.logger.error("Error in monitor")
                self.logger.error(f"{self.format_error(e)}")

    # STOP: オーバーライドの可能性なし
