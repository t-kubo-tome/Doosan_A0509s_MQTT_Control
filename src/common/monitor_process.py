import json
import logging
import logging.handlers
import os
import queue
import sys
import threading
import time
from enum import Enum, auto
import traceback
from typing import Any, Dict, List, TextIO

import psutil
from paho.mqtt import client as mqtt

from common.utils import AngleUnitConverter
from robot import config, MonitorHardware
from robot.shared_memory import NamedSharedMemory


class LoopResult(Enum):
    INTERRUPTED = auto()
    LOG_FILE_CHANGED = auto()


class MonitorProcess:
    """ロボットのモニタプロセス。"""

    # BEGIN: 汎用

    def __init__(self) -> None:
        self.hw: MonitorHardware | None = None
        self._angle_unit_converter = AngleUnitConverter(
            config.joint_unit_internal, config.joint_unit_external)

    def format_error(self, e: Exception) -> str:
        """例外をフォーマットする。"""
        if self.hw is not None:
            return self.hw.format_error(e)
        else:
            s = "Error trace: " + "\n" + traceback.format_exc()
        return s

    def del_robot(self) -> None:
        """ロボットのインスタンスを削除する。"""
        if self.hw is not None:
            self.hw.stop()
            self.hw.on_del()
            self.hw = None

    # END: 汎用

    # BEGIN: 受信コマンド

    def connect_robot(self) -> bool:
        """ロボットに接続する。例外の送出は禁止。"""
        try:
            # NOTE: robot_loggerをrun_processで初期化するため、
            # ControlHardwareを__init__内で初期化できないため、
            # このような実装にしている
            # NOTE: 再接続するためには、APIインスタンスの再生成
            # だけでなく、呼び出しプロセスの再起動も必要かもしれない
            if self.hw is None:
                self.hw = MonitorHardware(self)
            if not self.hw.start():
                raise ValueError("Failed to start robot")
            # NOTE: all_robot_stateはDoosanRobotExtの中に組み込めるかも
            self.all_robot_state = {}
            # ロボットによっては別のモニタプロセスで状態値を取得できないので
            # 制御プロセス中の別のスレッドで取得する
            if config.real_monitor_process == "monitor":
                self.init_monitor_loop()
            # NOTE: ハンドは現状、制御プロセスでモニタする
            return True
        except Exception as e:
            self.logger.error("Error in initializing robot: ")
            self.logger.error(f"{self.format_error(e)}")
            return False

    # END: 受信コマンド

    # BEGIN: 状態取得

    def get_current_pose_rt(self) -> List[float]:
        return self.hw.get_current_pose_rt()

    def get_current_joint_rt(self) -> List[float]:
        return self.hw.get_current_joint_rt()

    def get_current_force_rt(self) -> List[float]:
        return self.hw.get_current_force_rt()

    def get_all_robot_state_at_once(self) -> None:
        return self.hw.get_all_robot_state_at_once()

    def get_enabled(self) -> bool:
        return self.hw.get_enabled()

    def get_is_in_servo_mode(self) -> bool:
        return self.hw.get_is_in_servo_mode()

    def get_is_emergency_stopped(self) -> bool:
        return self.hw.get_is_emergency_stopped()

    def get_errors(self) -> List[Dict[str, Any]]:
        return self.hw.get_errors()

    # END: 状態取得

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

    def init_monitor_loop(self) -> None:
        self.monitor_thread = threading.Thread(
            target=self.monitor_loop)
        self.monitor_thread.start()
    
    def del_monitor_loop(self) -> None:
        if hasattr(self, 'monitor_thread'):
            self.monitor_thread.join()

    def monitor_loop(self) -> None:
        last = 0
        last_error_monitored = 0
        last_enabled = None
        last_is_in_servo_mode = None
        last_is_emergency_stopped = None
        last_health_check = 0
        while True:
            now = time.time()
            if last == 0:
                last = now
            if last_error_monitored == 0:
                last_error_monitored = now
            if last_health_check == 0:
                last_health_check = now
            if last_health_check + 60 < now:
                last_health_check = now
                self.logger.info("Health check: Robot monitor is running")

            actual_joint_js = {}

            # TCP姿勢
            actual_tcp_pose = None
            try:
                actual_tcp_pose = self.get_current_pose_rt()
            except Exception as e:
                self.logger.error(f"{self.format_error(e)}")

            # 関節
            actual_joint = None
            try:
                actual_joint = self.get_current_joint_rt()
            except Exception as e:
                self.logger.error(f"{self.format_error(e)}")

            # 起動時など両方0になるときがあるがそのような場合は無効なデータが入っている
            if actual_tcp_pose is not None and actual_joint is not None:
                if sum(actual_tcp_pose) == 0 and sum(actual_joint) == 0:                
                    actual_tcp_pose = None
                    actual_joint = None

            if actual_joint is not None:
                self.shm.joint_state = actual_joint
                self.shm.is_joint_state_received = 1
                actual_joint_js["joints"] = self.real_to_vr_joint(actual_joint)

            if actual_tcp_pose is not None:
                self.shm.pose_state = actual_tcp_pose
                self.shm.is_pose_state_received = 1
                actual_joint_js["poses"] = actual_tcp_pose

            actual_joint_js["time"] = now

            # [X, Y, Z, RX, RY, RZ]: センサ値の力[N]とモーメント[Nm]
            forces = None
            try:
                forces = self.get_current_force_rt()
            except Exception as e:
                self.logger.error(f"{self.format_error(e)}")
            if forces is not None:
                actual_joint_js["forces"] = forces

            # TODO: tool            

            # 1つの関数で複数の情報をまとめて取得する場合
            try:
                self.get_all_robot_state_at_once()
            except Exception as e:
                self.logger.error(f"{self.format_error(e)}")

            # モーターの電源がONか
            enabled = False
            try:
                enabled = self.get_enabled()
            except Exception as e:
                self.logger.error(f"{self.format_error(e)}")
            # 切り替わるときにログを出す
            if enabled != last_enabled:
                if enabled:
                    self.logger.info("Robot is enabled")
                else:
                    self.logger.info("Robot is disabled")
            last_enabled = enabled
            actual_joint_js["enabled"] = enabled

            # スレーブモードかどうかを取得する
            is_in_servo_mode = False
            try:
                is_in_servo_mode = self.get_is_in_servo_mode()
            except Exception as e:
                self.logger.error(f"{self.format_error(e)}")
            # 切り替わるときにログを出す
            if  is_in_servo_mode != last_is_in_servo_mode:
                if is_in_servo_mode:
                    self.logger.info("Robot is in servo mode")
                else:
                    self.logger.info("Robot is not in servo mode")
            last_is_in_servo_mode = is_in_servo_mode
            actual_joint_js["servo_mode"] = is_in_servo_mode
            self.shm.slave_mode = int(is_in_servo_mode)

            # 緊急停止状態かどうかを取得する
            is_emergency_stopped = False
            try:
                is_emergency_stopped = self.get_is_emergency_stopped()
            except Exception as e:
                self.logger.error(f"{self.format_error(e)}")
            # 切り替わるときにログを出す
            if is_emergency_stopped != last_is_emergency_stopped:
                if is_emergency_stopped:
                    self.logger.error("Emergency stop is ON")
                else:
                    self.logger.info("Emergency stop is OFF")
            last_is_emergency_stopped = is_emergency_stopped
            actual_joint_js["emergency_stopped"] = is_emergency_stopped
            self.shm.is_emergency_stopped = int(is_emergency_stopped)

            # エラー情報を取得する
            error = {}
            errors = []
            try:
                errors = self.get_errors()
            except Exception as e:
                self.logger.error(f"{self.format_error(e)}")
            if len(errors) > 0:
                error = {"errors": errors}
            # エラーがあるときだけ状態値として配信する
            if error:
                actual_joint_js["error"] = error

            # MQTT制御状態かどうかを取得する
            actual_joint_js["mqtt_control"] = self.shm.is_mqtt_control == 1

            # 状態値として配信用
            self.monitor_queue.put(actual_joint_js)

            if self.shm.exit_program == 1:
                break

            # 適度に間隔を開ける
            t_elapsed = time.time() - now
            t_wait = config.t_intv - t_elapsed
            if t_wait > 0:
                time.sleep(t_wait)

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
        self.client.connect(config.mqtt_server, 1883, 60)
        self.client.loop_start()

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
                    self.client.publish(config.mqtt_robot_state_topic, jss)
                    actual_joint_js["topic"] = config.mqtt_robot_state_topic
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
        self.shm = NamedSharedMemory(create=False)
        self.topic_memory = topic_memory
        self.slave_mode_lock = slave_mode_lock
        self.monitor_pipe = monitor_pipe
        self.monitor_queue = monitor_queue
        self.logging_dir = logging_dir

        try:
            self.init_realtime()
            self.connect_robot()
            self.connect_mqtt(disable_mqtt=disable_mqtt)
            # 処理ループ
            while True:
                if config.save:
                    with open(
                        os.path.join(self.logging_dir, "state.jsonl"), "a"
                    ) as f:
                        res = self.monitor_start(f)
                else:
                    res = self.monitor_start()
                if res == LoopResult.LOG_FILE_CHANGED:
                    self.get_logging_dir_and_change_log_file()
                elif res == LoopResult.INTERRUPTED:
                    break
        except Exception as e:
            self.logger.error("Error in monitor:")
            self.logger.error(f"{self.format_error(e)}")
        finally:
            self.del_robot()
            self.logger.info("Robot disconnected")
            if self.client is not None:
                self.client.loop_stop()
                self.client.disconnect()
            self.logger.info("MQTT client disconnected")
            self.monitor_queue.close()
            # 他のプロセスにも周知
            self.shm.exit_program = 1
            time.sleep(1)
            self.shm.release()
            self.logger.info("Shared memory released")
            self.logger.info("Process stopped")
            time.sleep(1)
            self.handler.close()
            self.robot_handler.close()
