# Doosanの状態をモニタリングする

from enum import auto, Enum
import logging
from typing import Any, Dict, List, TextIO
from paho.mqtt import client as mqtt

import time

import os
import sys
import json
import psutil

import multiprocessing as mp

import numpy as np

from dotenv import load_dotenv

from .config import SHM_NAME, SHM_SIZE, T_INTV
from .doosan_tools import tool_infos, tool_classes
# Robot specific modules


# パラメータ
load_dotenv(os.path.join(os.path.dirname(__file__),'.env'))
ROBOT_IP = os.getenv("ROBOT_IP", "192.168.5.45")
HAND_IP = os.getenv("HAND_IP", "192.168.5.46")
ROBOT_UUID = os.getenv("ROBOT_UUID","ur-real")
MQTT_SERVER = os.getenv("MQTT_SERVER", "sora2.uclab.jp")
MQTT_ROBOT_STATE_TOPIC = os.getenv("MQTT_ROBOT_STATE_TOPIC", "robot")+"/"+ROBOT_UUID
MQTT_MODE = os.getenv("MQTT_MODE", "metawork")
SAVE = os.getenv("SAVE", "true") == "true"

# 基本的に運用時には固定するパラメータ
save_state = SAVE


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
            if self.pose[34] == 1:
                return LoopResult.LOG_FILE_CHANGED

            actual_joint_js = self.monitor_queue.get()
            now = actual_joint_js["time"]
            if last == 0:
                last = now

            if now-last > 0.3 or "tool_change" in actual_joint_js or "put_down_box" in actual_joint_js:
                if self.client is not None:
                    jss = json.dumps(actual_joint_js)
                    self.client.publish(MQTT_ROBOT_STATE_TOPIC, jss)
                    actual_joint_js["topic_type"] = "robot"
                    actual_joint_js["topic"] = MQTT_ROBOT_STATE_TOPIC
                with self.monitor_lock:
                    self.monitor_dict.clear()
                    self.monitor_dict.update(actual_joint_js)
                last = now

            # MQTT手動制御モード時のみ記録する
            # それ以外の時のエラーはstate情報は必要ないと考えたため
            if f is not None and self.pose[15] == 1:
                datum = dict(
                    time=now,
                    kind="state",
                    joint=actual_joint_js.get("joints"),
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
            if self.pose[32] == 1:
                return LoopResult.INTERRUPTED

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
        self.pose[34] = 0

    def run_proc(self, monitor_dict, monitor_lock, slave_mode_lock, log_queue, monitor_pipe, monitor_queue, logging_dir, disable_mqtt: bool = False):
        self.setup_logger(log_queue)
        self.logger.info("Process started")
        self.sm = mp.shared_memory.SharedMemory(SHM_NAME)
        self.pose = np.ndarray((SHM_SIZE,), dtype=np.dtype("float32"), buffer=self.sm.buf)
        self.monitor_dict = monitor_dict
        self.monitor_lock = monitor_lock
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
                    self.sm.close()
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
