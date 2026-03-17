# Doosanの状態をモニタリングする
import json
import logging
import logging.handlers
import os
import queue
import sys
import time
from enum import Enum, auto
from typing import Any, Dict, List, TextIO

import psutil
from paho.mqtt import client as mqtt

from ..common.utils import rad2deg_list
from .config import (
    HAND_IP,
    MQTT_MODE,
    MQTT_ROBOT_STATE_TOPIC,
    MQTT_SERVER,
    ROBOT_IP,
    ROBOT_UUID,
    SAVE,
)
from .shared_memory import NamedSharedMemory

# 基本的に運用時には固定するパラメータ
save_state = SAVE
mqtt_robot_state_topic = MQTT_ROBOT_STATE_TOPIC + "/" + ROBOT_UUID


class LoopResult(Enum):
    NOT_CONNECTED = auto()
    INTERRUPTED = auto()
    LOG_FILE_CHANGED = auto()


class Doosan_MON:
    def __init__(self):
        pass

    def format_error(self, e: Exception) -> str:
        return str(e)

    def init_robot(self):
        pass

    def find_and_setup_hand(self, tool_id):
        pass

    def reconnect_robot(self):
        pass

    def disconnect_robot(self):
        pass

    def reconnect_after_timeout(self, e: Exception) -> bool:
        pass

    def init_realtime(self):
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

    def on_connect(self, client, userdata, connect_flags, reason_code, properties):
        # 接続できた旨表示
        self.logger.info("MQTT connected with result code: " + str(reason_code))
        
    def on_disconnect(
        self,
        client,
        userdata,
        disconnect_flags,
        reason_code,
        properties,
    ):
        if reason_code != 0:
            self.logger.warning("MQTT unexpected disconnection.")

    def connect_mqtt(self, disable_mqtt: bool = False):
        if disable_mqtt:
            self.client = None
            return
        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect         # 接続時のコールバック関数を登録
        self.client.on_disconnect = self.on_disconnect   # 切断時のコールバックを登録
        self.client.connect(MQTT_SERVER, 1883, 60)
        self.client.loop_start()   # 通信処理開始

    def get_tool_info(
        self, tool_infos: List[Dict[str, Any]], tool_id: int) -> Dict[str, Any]:
        return [tool_info for tool_info in tool_infos
                if tool_info["id"] == tool_id][0]

    def monitor_start(self, f: TextIO | None = None) -> LoopResult:
        # ロボット固有の処理を含む
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
                actual_joint_js = self.monitor_queue.get(
                    block=True, timeout=1)
            except queue.Empty:
                continue

            now = actual_joint_js["time"]
            if last == 0:
                last = now

            if now-last > 0.3 or "tool_change" in actual_joint_js or "put_down_box" in actual_joint_js:
                if self.client is not None:
                    jss = json.dumps(actual_joint_js)
                    self.client.publish(mqtt_robot_state_topic, jss)
                    actual_joint_js["topic"] = mqtt_robot_state_topic
                self.topic_memory.write("robot", actual_joint_js)
                last = now

            # MQTT手動制御モード時のみ記録する
            # それ以外の時のエラーはstate情報は必要ないと考えたため
            if f is not None and self.shm.is_mqtt_control == 1:
                joints = actual_joint_js.get("joints")
                if joints is not None:
                    joints = rad2deg_list(joints)
                datum = dict(
                    time=now,
                    kind="state",
                    joint=joints,
                    pose=actual_joint_js.get("poses"),
                    # width=width,
                    # force=force,
                    forces=actual_joint_js.get("forces"),
                    error=actual_joint_js.get("error", {}),
                    enabled=actual_joint_js["enabled"],
                    # TypeError: Object of type float32 is not JSON
                    # serializableへの対応
                    # tool_id=float(tool_id),
                    # other=info,
                )
                js = json.dumps(datum, ensure_ascii=False)
                f.write(js + "\n")

    def setup_logger(self, log_queue):
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

    def run_proc(self, topic_memory, slave_mode_lock, log_queue, monitor_pipe, monitor_queue, logging_dir, disable_mqtt: bool = False):
        self.setup_logger(log_queue)
        self.logger.info("Process started")
        self.shm = NamedSharedMemory(create=False)
        self.topic_memory = topic_memory
        self.slave_mode_lock = slave_mode_lock
        self.monitor_pipe = monitor_pipe
        self.monitor_queue = monitor_queue
        self.logging_dir = logging_dir

        self.init_realtime()
        self.init_robot()
        self.connect_mqtt(disable_mqtt=disable_mqtt)
        while True:
            try:
                if save_state:
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



if __name__ == '__main__':
    cp = Doosan_MON()
    cp.init_realtime()
    cp.init_robot()
    cp.connect_mqtt()

    try:
        cp.monitor_start()
    except KeyboardInterrupt:
        print("Monitor Main Stopped")
        # cp.robot.disable()
        # cp.robot.stop()
