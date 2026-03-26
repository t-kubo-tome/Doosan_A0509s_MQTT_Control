import logging
import os
import sys
import threading
import time
import traceback
from typing import Any, Dict, List

import modern_robotics as mr
import numpy as np
import psutil

from common.filter import SMAFilter
from common.interpolate import DelayedInterpolator
from common.utils import StopWatch
from robot import config, ControlHardware
from robot.shared_memory import NamedSharedMemory
from robot.tools import tool_classes, tool_infos


class ControlProcess:
    """ロボットの制御プロセス。"""

    # BEGIN: 汎用

    def __init__(self) -> None:
        # NOTE: robot_loggerをrun_processで初期化するため、
        # ControlHardwareを__init__内で初期化できないため、
        # このような実装にしている
        self.hw: ControlHardware | None = None

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
            self.hw.disable()
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
                self.hw = ControlHardware(self)
            if not self.hw.start():
                raise ValueError("Failed to start robot")
            # NOTE: all_robot_stateはDoosanRobotExtの中に組み込めるかも
            self.all_robot_state = {}
            # ロボットによっては別のモニタプロセスで状態値を取得できないので
            # 制御プロセス中の別のスレッドで取得する
            self.init_monitor_loop()
            # NOTE: 接続を何度も可能にする場合、ツールが複数ある場合は、
            # 最初のツールIDではだめな場合がある
            tool_id = int(os.environ["TOOL_ID"])
            # NOTE: ここでハンドの接続に成功したかどうか判定してもよい
            # その場合ハンドの接続に成功しなくても使えるオプションを設けるとよい
            self.find_and_setup_hand(tool_id)
            return True
        except Exception as e:
            self.logger.error("Error in initializing robot: ")
            self.logger.error(f"{self.format_error(e)}")
            return False

    def enable(self) -> bool:
        """ロボットのモーターの電源をONにする。例外の送出は禁止。"""
        self.logger.info("Enabling robot")
        try:
            if not self.hw.enable():
                raise ValueError("Failed to enable robot")
            return True
        except Exception as e:
            self.logger.error("Error enabling robot")
            self.logger.error(f"{self.format_error(e)}")
            return False

    def disable(self) -> bool:
        """ロボットのモーターの電源をOFFにする。例外の送出は禁止。"""
        self.logger.info("Disabling robot")
        try:
            if not self.hw.disable():
                raise ValueError("Failed to disable robot")
            return True
        except Exception as e:
            self.logger.error("Error disabling robot")
            self.logger.error(f"{self.format_error(e)}")
            return False

    def set_area_enabled(self, enable: bool) -> bool:
        """ロボットのエリア制限をON/OFFする。例外の送出は禁止。"""
        self.logger.info(f"Setting area enabled: {enable}")
        try:
            if not self.hw.set_area_enabled(enable):
                raise ValueError("Failed to set area enabled")
        except Exception as e:
            self.logger.error("Error setting area enabled")
            self.logger.error(f"{self.format_error(e)}")
            return False

    def tidy_pose(self) -> bool:
        """ロボットをデフォルトの姿勢に移動させる。例外の送出は禁止。"""
        self.logger.info("Tidy pose")
        try:
            if not self.hw.move_joint(*config.tidy_joint):
                raise ValueError("Failed to move to tidy pose")
            return True
        except Exception as e:
            self.logger.error("Error moving to tidy pose")
            self.logger.error(f"{self.format_error(e)}")
            return False

    def move_joint(self, joints: List[float]) -> bool:
        """ロボットを関節空間で移動させる。例外の送出は禁止。"""
        self.logger.info("Move joint")
        try:
            ret = self.hw.move_joint(*joints)
            if not ret:
                raise ValueError("Failed to move to joint pose")
            return True
        except Exception as e:
            self.logger.error("Error moving to joint pose")
            self.logger.error(f"{self.format_error(e)}")
            return False

    def clear_error(self) -> bool:
        """ロボットのエラーをクリアする。例外の送出は禁止。"""
        self.logger.info("Clear error")
        try:
            if not self.hw.clear_error():
                raise ValueError("Failed to clear error")
            return True
        except Exception as e:
            self.logger.error("Error clearing error")
            self.logger.error(f"{self.format_error(e)}")
            return False

    def jog_joint(self, joint: int, direction: float) -> bool:
        """関節をジョグする。例外の送出は禁止。"""
        try:
            if self.shm.is_joint_state_received != 1:
                raise ValueError("Joint jog requires joint state to be monitored but currently not")
            # joint state
            joints = self.shm.joint_state.copy()
            joints = np.asarray(joints)
            joints[joint] += direction
            joints = joints.tolist()
            is_success = self.hw.move_joint(*joints)
            if not is_success:
                raise ValueError("move_joint failed")
            return True
        except Exception as e:
            self.logger.error("Error during joint jog")
            self.logger.error(f"{self.format_error(e)}")
            return False

    def jog_tcp(self, axis: int, direction: float) -> bool:
        """TCPをジョグする。例外の送出は禁止。"""
        try:
            if self.shm.is_pose_state_received != 1:
                raise ValueError("TCP jog requires TCP state to be monitored but currently not")
            # TCP state
            poses = self.shm.pose_state.copy()
            poses = np.asarray(poses)
            poses[axis] += direction
            poses = poses.tolist()
            is_success = self.hw.move_pose(*poses)
            if not is_success:
                raise ValueError("move_pose failed")
            return True
        except Exception as e:
            self.logger.error("Error during TCP jog")
            self.logger.error(f"{self.format_error(e)}")
            return False

    # END: 受信コマンド

    # BEGIN: MQTT制御

    def enter_servo_mode(self) -> None:
        """ロボットをサーボモードに切り替える。"""
        # self.shm.maybe_slave_modeは0のとき必ず通常モード。
        # self.shm.maybe_slave_modeは1のとき基本的にスレーブモードだが、
        # 変化前後の短い時間は通常モードの可能性がある。
        # 順番固定
        with self.slave_mode_lock:
            self.shm.maybe_slave_mode = 1
            self.hw.enter_servo_mode()

    def leave_servo_mode(self) -> None:
        """ロボットをサーボモードから離れる。"""
        # self.shm.maybe_slave_modeは0のとき必ず通常モード。
        # self.shm.maybe_slave_modeは1のとき基本的にスレーブモードだが、
        # 変化前後の短い時間は通常モードの可能性がある。
        # 順番固定
        self.hw.leave_servo_mode()
        self.shm.maybe_slave_mode = 0

    def should_recover_automatic_on_timeout_error(self, e_leave) -> bool:
        """タイムアウトエラーが発生したときに自動的に回復するかどうか。例外の送出は禁止。"""
        self.logger.info("Should recover automatic on timeout error")
        try:
            ret = self.hw.should_recover_automatic_on_timeout_error(e_leave)
            if not ret:
                raise ValueError("Failed to determine if automatic recovery is possible on timeout error")
            return True
        except Exception as e:
            self.logger.error("Error during automatic recovery check on timeout error")
            self.logger.error(f"{self.format_error(e)}")
            return False

    def recover_automatic_on_timeout_error(self) -> bool:
        """タイムアウトエラーが発生したときに自動的に回復する処理を行う。例外の送出は禁止。"""
        self.logger.info("Attempting automatic recover from timeout error")
        try:
            ret = self.hw.recover_automatic_on_timeout_error()
            if not ret:
                raise ValueError("Failed to recover from timeout error")
            return True
        except Exception as e:
            self.logger.error("Error during automatic recover on timeout error")
            self.logger.error(f"{self.format_error(e)}")
            return False

    def recover_automatic_on_recoverable_error(self) -> bool:
        """回復可能なエラーが発生したときに自動的に回復する処理を行う。例外の送出は禁止。"""
        self.logger.info("Attempting automatic recover from recoverable error")
        try:
            ret = self.hw.recover_automatic_on_recoverable_error()
            if not ret:
                raise ValueError("Failed to recover from recoverable error")
            return True
        except Exception as e:
            self.logger.error("Error during automatic recover on recoverable error")
            self.logger.error(f"{self.format_error(e)}")
            return False

    def on_step_start_in_control_loop(self) -> None:
        self.hw.on_step_start_in_control_loop()

    def should_wait_control_loop(self) -> bool:
        self.hw.should_wait_control_loop()

    def is_ready_to_stop(self) -> bool:
        # スレーブモードでは十分低速時に2回同じ位置のコマンドを送ると
        # ロボットを停止させてスレーブモードを解除可能な状態になる
        # Cobotta Proに必要な処理であるが、他のロボットにあっても
        # よいため使用している
        return (self.control == self.last_control).all()

    def move_joint_servo(
        self,
        control: List[float],
        lock,
        error_info,
        error_event,
        stop_event,
    ) -> bool:
        """関節のスレーブモードでの制御値をロボットに送る。例外の送出は禁止。"""
        try:
            if not self.hw.move_joint_servo(*control):
                raise ValueError("Failed to send servoJ command")
            return True
        except Exception as e:
            with lock:
                error_info['kind'] = "robot"
                error_info['msg'] = str(e)
                error_info['exception'] = e
            error_event.set()
            stop_event.set()
            return False

    def move_joint_servo_by_vel(
        self,
        control: List[float],
        lock,
        error_info,
        error_event,
        stop_event,
    ) -> bool:
        """関節のスレーブモードでの速度制御値をロボットに送る。例外の送出は禁止。"""
        try:
            if not self.hw.move_joint_servo_by_vel(*control):
                raise ValueError("Failed to send servoJ command")
            return True
        except Exception as e:
            with lock:
                error_info['kind'] = "robot"
                error_info['msg'] = str(e)
                error_info['exception'] = e
            error_event.set()
            stop_event.set()
            return False

    # END: MQTT制御

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

    def init_monitor_loop(self) -> None:
        self.monitor_thread = threading.Thread(
            target=self.monitor_loop)
        self.monitor_thread.start()
    
    def del_monitor_loop(self) -> None:
        if hasattr(self, 'monitor_thread'):
            self.monitor_thread.join()

    def monitor_loop(self) -> None:
        # TODO: monitor_loop_stepを実装してもいいかもしれない
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

    def hand_control_loop(
        self, stop_event, error_event, lock, error_info,
    ) -> None:
        self.logger.info("Start Hand Control Loop")
        last_tool_corrected = None
        t_intv_hand = config.t_intv * 2
        last_tool_corrected_time = time.time()
        while True:
            now = time.time()
            if stop_event.is_set():
                break
            # 現在情報を取得しているかを確認
            if self.shm.is_joint_state_received != 1:
                time.sleep(t_intv_hand)
                continue
            # 目標値を取得しているかを確認
            if self.shm.is_joint_target_received != 1:
                time.sleep(t_intv_hand)
                continue
            # ツールの値を取得
            # 値0が意味を持つので共有メモリではオフセットをかけている
            tool = self.shm.hand_target
            if tool == 0:
                time.sleep(t_intv_hand)
                continue
            tool_corrected = tool
            if tool_corrected != last_tool_corrected:
                if tool_corrected == 1:
                    self.logger.info("Send grip command to hand")
                    is_success = self.send_grip(
                        lock, error_info, error_event, stop_event)
                    if not is_success:
                        break
                elif tool_corrected == 2:
                    self.logger.info("Send release command to hand")
                    is_success = self.send_release(lock, error_info, error_event, stop_event)
                    if not is_success:
                        break
            # ハンドの状態値を取得
            # 情報を常に取得するとアームの制御ループの処理間隔を乱し
            # 情報が必要なのはハンドに制御値を送った少し後だけなので以下のようにする
            # NOTE: 常に取得した方がいいかもしれない。処理間隔を乱すなら別プロセス化の方がいいかもしれない
            if now - last_tool_corrected_time < 1:
                is_success = self.get_hand_state(
                    lock, error_info, error_event, stop_event)
                if not is_success:
                    break
            if tool_corrected != last_tool_corrected:
                last_tool_corrected = tool_corrected
                last_tool_corrected_time = now
            # 適度に間隔を開ける
            t_elapsed = time.time() - now
            t_wait = t_intv_hand - t_elapsed
            if t_wait > 0:
                time.sleep(t_wait)
        self.logger.info("Stop Hand Control Loop")

    def stop_by_user_or_emergency_before_sending_control(
        self, stop, stop_event, error_event, lock, error_info
    ) -> bool:
        # ロボットに制御値を送る前に、ユーザーが停止を要求した場合、即時終了可能
        if stop:
            # ハンドの制御を止める
            stop_event.set()
            return True
        # ロボットに制御値を送る前は、非常停止が押されているかどうかは、
        # 制御値を送るコマンドのエラーで捕捉できないので、
        # スレーブモードが解除されているかで確認する
        if self.shm.slave_mode != 1:
            msg = "Robot is not in servo mode"
            with lock:
                error_info['kind'] = "robot"
                error_info['msg'] = msg
                error_info['exception'] = ValueError(msg)
            error_event.set()
            stop_event.set()
            return True
        return False

    def speed_limit(
        self,
        dt: float,
        v: np.ndarray,
        v_last: np.ndarray,
        v_limits: np.ndarray,
        a_limits: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, float, float]:
        # 速度制限
        ratio = np.abs(v) / v_limits
        max_ratio = np.max(ratio)
        if max_ratio > 1:
            v /= max_ratio

        # 加速度制限
        a = (v - v_last) / dt
        accel_ratio = np.abs(a) / a_limits
        accel_max_ratio = np.max(accel_ratio)
        if accel_max_ratio > 1:
            a /= accel_max_ratio
        v = v_last + a * dt

        return v, max_ratio, accel_max_ratio

    def try_first_speed_limit(
        self,
        target_diff: np.ndarray,
        dt: float,
        last_target_delayed_velocity: np.ndarray,
        use_first_speed_limit: bool,
    ) -> tuple[np.ndarray, float, float]:
        first_max_ratio = -1
        first_accel_max_ratio = -1

        if use_first_speed_limit:
            v = target_diff / dt
            v, first_max_ratio, first_accel_max_ratio = self.speed_limit(
                dt, v, last_target_delayed_velocity,
                config.eff_speed_limits, config.eff_accel_limits,
            )
            target_diff = v * dt

        # 速度がしきい値より小さければ静止させ、ドリフトや振動を避ける
        # NOTE: どのロボットにも有意義な処理である。特にCobotta Proの
        # スレーブモードを正常に解除するためにも必要
        if np.all(v < config.stopped_velocity_eps):
            target_diff = np.zeros(config.n_joints)

        return target_diff, first_max_ratio, first_accel_max_ratio

    def try_second_speed_limit(
        self,
        target_diff: np.ndarray,
        dt: float,
        last_control_velocity: np.ndarray,
        use_second_speed_limit: bool,
        filter_kind: str,
    ) -> tuple[np.ndarray, float, float]:
        if filter_kind == "feedback_pd_traj":
            return target_diff, -1, -1
        max_ratio = -1
        accel_max_ratio = -1

        if use_second_speed_limit:
            v = target_diff / dt
            v, max_ratio, accel_max_ratio = self.speed_limit(
                dt, v, last_control_velocity,
                config.eff_speed_limits, config.eff_accel_limits,
            )
            target_diff = v * dt

        # 速度がしきい値より小さければ静止させ、ドリフトや振動を避ける
        # NOTE: どのロボットにも有意義な処理である。特にCobotta Proの
        # スレーブモードを正常に解除するためにも必要
        if np.all(v < config.stopped_velocity_eps):
            target_diff = np.zeros(config.n_joints)

        return target_diff, max_ratio, accel_max_ratio

    def init_filter(
        self,
        filter_kind: str,
        n_windows: int, 
        state: np.ndarray,
        target: np.ndarray,
    ) -> None:
        if filter_kind == "original":
            self._filter = SMAFilter(n_windows=n_windows)
            self._filter.reset(state)
        elif filter_kind == "target":
            self._filter = SMAFilter(n_windows=n_windows)
            self._filter.reset(target)
        elif filter_kind == "filter_target_from_target_but_diff_from_control":
            self._filter = SMAFilter(n_windows=n_windows)
            self._filter.reset(target)
        elif filter_kind == "state_and_target_diff":
            self._filter = SMAFilter(n_windows=n_windows)
            self._filter.reset(state)
        elif filter_kind == "moveit_servo_humble":
            self._filter = SMAFilter(n_windows=n_windows)
            self._filter.reset(state)
        elif filter_kind == "control_and_target_diff":
            self._filter = SMAFilter(n_windows=n_windows)
            self._filter.reset(state)
        elif filter_kind == "feedback_pd_traj":
            N = config.n_joints
            Tf = config.t_intv * (N - 1)
            method = 5
            Kp = 0.6
            Kd = 0.02
            prev_error = np.zeros(config.n_joints)
            pd_step = 0
            self._filter_params = {
                "N": N,
                "Tf": Tf,
                "method": method,
                "Kp": Kp,
                "Kd": Kd,
                "prev_error": prev_error,
                "pd_step": pd_step,
                "last_control_velocity": np.zeros((N - 1, config.n_joints)),
            }
        elif filter_kind == "none":
            pass

    def try_filter(
        self, target_delayed: np.ndarray, state: np.ndarray, filter_kind: str,
    ) -> tuple[np.ndarray, np.ndarray]:
        # 平滑化
        if filter_kind == "filter_target_from_target_but_diff_from_control":
            # 成功している方法
            target_filtered = self._filter.filter(target_delayed)
            target_filtered_base = self.last_control
            target_diff = target_filtered - target_filtered_base
        elif filter_kind == "original":
            # 成功している方法
            # 速度制限済みの制御値で平滑化をしており、
            # moveit servoなどでは見られない処理
            target_filtered = self._filter.predict_only(target_delayed)
            target_filtered_base = self.last_control
            target_diff = target_filtered - target_filtered_base
        elif filter_kind == "target":
            # 成功することもあるが平滑化窓を増やす必要あり
            # 状態値を無視した目標値の値をロボットに送る
            last_target_filtered = self._filter.previous_filtered_measurement
            target_filtered = self._filter.filter(target_delayed)
            target_filtered_base = last_target_filtered
            target_diff = target_filtered - target_filtered_base
        elif filter_kind == "state_and_target_diff":
            # 失敗する
            # 状態値に目標値の差分を足したものを平滑化する
            # moveit servo (少なくともhumble版)ではこのようにしているが、
            # 速度がどんどん大きくなっていって（正のフィードバック）
            # 制限に引っかかる
            # stateを含む移動平均を取ると、stateが速度を持つと
            # その速度を保持し続けようとするので、そこに差分を足すと
            # どんどん加速していくのでは。
            # 遅延があることも影響しているかも。
            # NOTE(20250813): targetがstateのフィードバックを受けていない場合は
            # target_alignedは、stateとtargetが乖離するので不適切
            target_diff = target_delayed - self.last_target_delayed
            target_aligned = state + target_diff
            last_target_filtered = self._filter.previous_filtered_measurement
            target_filtered = self._filter.filter(target_aligned)
            target_filtered_base = last_target_filtered
            target_diff = target_filtered - target_filtered_base
        elif filter_kind == "moveit_servo_humble":
            # 失敗する
            # 停止はしないがかなりゆっくり動き、目標軌跡も追従しなくなる
            # v = (target_filtered - state) / t_intv
            # target_filteredとstateの差は、
            # - 制御値を送ってからその値にstateがなるまで0.1s程度の遅延があること
            # - テストなどであらかじめ決まっているtargetを逐次送り、
            #   targetの速度がロボットの速度制限より大きいとき、
            #   targetがstateからどんどん離れていくこと
            # などの理由からt_intv秒で移動できる距離以上になってしまうため
            target_diff = target_delayed - self.last_target_delayed
            target_aligned = state + target_diff
            last_target_filtered = self._filter.previous_filtered_measurement
            target_filtered = self._filter.filter(target_aligned)
            target_filtered_base = state
            target_diff = target_filtered - target_filtered_base
        elif filter_kind == "control_and_target_diff":
            # 失敗する
            # 速度制限にひっかかり途中停止する
            # 制御値に目標値の差分を足したものを平滑化する
            # 上記と同様に正のフィードバック的になっている
            # moveit_servo_mainの処理に近い
            target_diff = target_delayed - self.last_target_delayed
            target_aligned = self.last_control + target_diff
            last_target_filtered = self._filter.previous_filtered_measurement
            target_filtered = self._filter.filter(target_aligned)
            target_filtered_base = last_target_filtered
            target_diff = target_filtered - target_filtered_base
        elif filter_kind == "feedback_pd_traj":
            N = self._filter_params["N"]
            Tf = self._filter_params["Tf"]
            method = self._filter_params["method"]
            Kp = self._filter_params["Kp"]
            Kd = self._filter_params["Kd"]
            prev_error = self._filter_params["prev_error"]
            pd_step = self._filter_params["pd_step"]
            last_control_velocity = self._filter_params["last_control_velocity"]
            if pd_step == 0:
                error = target_delayed - state
                d_error = (error - prev_error) / Tf
                # 現状使用しない
                # mse = np.mean(error ** 2)
                target_goal = state + Kp * error + Kd * d_error
                prev_error = error
                self._filter_params["prev_error"] = prev_error
                target_steps = mr.JointTrajectory(
                    state.tolist(),
                    target_goal.tolist(),
                    Tf,
                    N,
                    method,
                )
                # 速度制限
                dt = config.t_intv
                # [N - 1, config.n_joints]
                target_diffs = np.diff(target_steps, axis=0)
                vs = target_diffs / dt
                ratios = np.abs(vs) / config.eff_speed_limits[None, :]
                max_ratio = np.max(ratios)
                if max_ratio > 1:
                    vs /= max_ratio

                # 加速度制限
                # [N, config.n_joints]
                vs_ = np.concatenate([last_control_velocity[[-1], :], vs], axis=0)
                # [N - 1, config.n_joints]
                as_ = np.diff(vs_, axis=0) / dt
                accel_ratios = np.abs(as_) / config.eff_accel_limits[None, :]
                accel_max_ratio = np.max(accel_ratios)
                if accel_max_ratio > 1:
                    as_ /= accel_max_ratio
                # [N - 1, config.n_joints]
                vs_ = vs_[0][None, :] + np.cumsum(as_, axis=0) * dt

                target_diffs_speed_limited = vs_ * dt
                # 速度がしきい値より小さければ静止させ、ドリフトや振動を避ける
                # NOTE: どのロボットにも有意義な処理である。特にCobotta Proの
                # スレーブモードを正常に解除するためにも必要
                for i in range(N - 1):
                    if np.all(target_diffs_speed_limited[i] / dt < config.stopped_velocity_eps):
                        target_diffs_speed_limited[i] = np.zeros_like(
                            target_diffs_speed_limited[i])
                        vs_[i] = target_diffs_speed_limited[i] / dt

                # [N - 1, config.n_joints]
                target_steps_speed_limited = target_steps[0][None, :] + np.cumsum(vs_, axis=0) * dt
                last_control_velocity = vs_
                self._filter_params["last_control_velocity"] = last_control_velocity

            target_filtered = target_steps_speed_limited[pd_step]
            target_filtered_base = target_filtered
            target_diff = np.zeros(config.n_joints)

            # Next step
            pd_step += 1
            if pd_step == N - 1:
                pd_step = 0
            self._filter_params["pd_step"] = pd_step
        elif filter_kind == "none":
            target_filtered = target_delayed
            target_filtered_base = self.last_control
            target_diff = target_filtered - target_filtered_base
        else:
            raise ValueError
        return target_diff, target_filtered_base

    def control_loop(self) -> bool:
        """リアルタイム制御ループ"""
        # ロボット固有の処理を含まない
        self.last = 0
        self.logger.info("Start Control Loop")
        # 状態値が最新の値になるようにする
        time.sleep(1)
        self.shm.is_joint_state_received = 0
        self.shm.is_joint_target_received = 0
        target_stop = None
        sw = StopWatch()
        stop_event = threading.Event()
        error_event = threading.Event()
        lock = threading.Lock()
        error_info = {}
        last_target = None

        # ハンドの制御は別スレッドで行う（同一スレッドで行うとアームの制御が
        # 遅くなる事例を複数ロボットで観測済みのため）
        hand_thread = threading.Thread(
            target=self.hand_control_loop,
            args=(stop_event, error_event, lock, error_info)
        )
        hand_thread.start()

        # アームの制御ループ
        while True:
            sw.start("Get shared memory")
            now = time.time()
            dt = now - self.last
            # 各ステップ開始時のロボット固有の処理
            self.on_step_start_in_control_loop()

            # ハンドの制御が止まった場合はアームの制御を即座に止める
            if not hand_thread.is_alive():
                break

            # ユーザーが停止を要求した場合
            stop = self.shm.stop_realtime_control

            # 状態値を取得しているかを確認
            if self.shm.is_joint_state_received != 1:
                # ロボットに制御値を送る前の停止判定
                if self.stop_by_user_or_emergency_before_sending_control(
                    stop, stop_event, error_event, lock, error_info
                ):
                    break
                # NOTE: 目標値の生成時に状態値を常に参照しない場合短い周期で確認しズレ防止
                time.sleep(config.t_intv)
                continue

            # ツールチェンジなどの後でリアルタイム制御が可能になったタイミングを
            # 知らせるためのフラグ
            self.shm.is_controllable = 1

            # 目標値を取得しているかを確認
            if self.shm.is_joint_target_received != 1:
                # ロボットに制御値を送る前の停止判定
                if self.stop_by_user_or_emergency_before_sending_control(
                    stop, stop_event, error_event, lock, error_info
                ):
                    break
                # NOTE: 目標値の生成時に状態値を常に参照しない場合短い周期で確認しズレ防止
                time.sleep(config.t_intv)
                continue

            # 関節の状態値
            state = self.shm.joint_state.copy()
            # 関節の目標値
            target = self.shm.joint_target.copy()

            # 目標値のチェック
            sw.lap("Check target")
            target_raw = target

            # NOTE: 目標値の角度が360度の不定性が許される場合
            #  (1度と-359度を区別しない場合、例：VR中の角度) でも、
            # 実機の関節の角度は360度の不定性が許されないので
            # 状態値に最も近い目標値に規格化する
            # TODO: VRと実機の関節の角度が360度の倍数だけずれた状態で、
            # 実機側で制限値を超えると動くVRと動かない実機との間に360度の倍数で
            # ないずれが生じ、急に実機が動く可能性があるので、VR側で
            # 実機との比較をし、実機側で制限値を超えることがないようにする必要がある。
            # あるいは目標値の角度範囲が実機と一致するようにするようきちんと
            # モデル化すればそもそも以下の処理は不要である。
            # なおとりあえず急に動こうとすれば止まる仕組みは入れている
            target_norm = target
            if config.use_normalize_target_to_nearest:
                target_norm = state + (target - state + 180) % 360 - 180
                target = target_norm

            # TODO: VR側でもソフトリミットを設定したほうが良い
            target_th = np.maximum(target, -config.abs_joint_soft_limit)
            if (target != target_th).any():
                self.logger.warning("target reached minimum threshold")
            target_th = np.minimum(target_th, config.abs_joint_soft_limit)
            if (target != target_th).any():
                self.logger.warning("target reached maximum threshold")
            target = target_th

            # 目標値が状態値から大きく離れた場合
            if (np.abs(target - state) > 
                config.target_state_abs_joint_diff_limit).any():
                # NOTE: まだ目標値を受け取っていない場合か、
                # すでに目標値を受け取っていて前回の目標値からも大きく離れていた場合のみ、
                # 停止させる
                # それ以外の場合は、目標値に比べて制御値、状態値の追従が遅れているだけで
                # ありえるので停止させない
                # VR側との同期方法が改善されれば不要な処理になる可能性あり
                if (
                    last_target is None or
                    (np.abs(target - last_target) > 
                    config.target_state_abs_joint_diff_limit).any()
                ):
                    # 強制停止する。ゆるやかな停止ではエラーの返し方が複雑になるため
                    msg = "Target and state are too different.\n"
                    msg += "| Joint  | Target | State  | Diff   | Limit  |\n"
                    msg += "| ------ | ------ | ------ | ------ | ------ |\n"
                    for i in range(config.n_joints):
                        if abs(target[i] - state[i]) > config.target_state_abs_joint_diff_limit[i]:
                            msg += f"| {i: <6d} | {target[i]: <6.1f} | {state[i]: <6.1f} | {abs(target[i] - state[i]): <6.1f} | {config.target_state_abs_joint_diff_limit[i]: <6.1f} |\n"
                    with lock:
                        error_info['kind'] = "robot"
                        error_info['msg'] = msg
                        error_info['exception'] = ValueError(msg)
                    error_event.set()
                    stop_event.set()
                    break

            last_target = target

            # 最初の目標値を受け取ったときの処理
            sw.lap("First target")
            if self.last == 0:
                self.logger.info("Start sending control command")
                self.last = now

                # ロボットに制御値を送る前の停止判定
                if self.stop_by_user_or_emergency_before_sending_control(
                    stop, stop_event, error_event, lock, error_info
                ):
                    break

                # 目標値を遅延を許して極力線形補間するためのセットアップ
                if config.use_interp:
                    di = DelayedInterpolator(delay=config.delay_for_interpolation)
                    di.reset(now, target)
                    target_delayed = di.read(now, target)
                else:
                    target_delayed = target


                self.last_target_delayed = target_delayed
                self.last_target_delayed_velocity = np.zeros(config.n_joints)

                # 制御値の初期化
                self.last_control = state
                self.last_control_velocity = np.zeros(config.n_joints)

                # 移動平均フィルタのセットアップ
                self.init_filter(config.filter_kind, config.n_windows, state, target)

                # 最初の目標値を受け取ったときは制御値は送らない
                continue

            sw.lap("Check stop")
            # ユーザーが停止を要求した場合、制御値を送信済みの場合は、
            # 要求時点での状態値を目標値に固定してロボットをゆるやかに静止させてから停止する
            # なお厳密にはここに始めて到達した時点では、制御値はまだ送っていないが、
            # 処理の簡潔さのために
            # 簡潔さのため同じように扱う
            if stop:
                stop_event.set()
                if target_stop is None:
                    target_stop = state
                target = target_stop

            sw.lap("Read delayed interpolator")
            # target_delayedは、delay秒前の目標値を前後の値を
            # 使って線形補間したもの
            if config.use_interp:
                target_delayed = di.read(now, target)
            else:
                target_delayed = target
            
            target_delayed_raw = target_delayed

            sw.lap("1st speed limit")
            # 速度制限を平滑化の前に入れ、制限された速度のスケールで平滑化できるようにする
            target_diff = target_delayed - self.last_target_delayed
            target_diff, first_max_ratio, first_accel_max_ratio = \
                self.try_first_speed_limit(
                    target_diff,
                    dt,
                    self.last_target_delayed_velocity,
                    config.use_first_speed_limit,
                )
            target_delayed = self.last_target_delayed + target_diff
            self.last_target_delayed = target_delayed
            self.last_target_delayed_velocity = target_diff / dt

            sw.lap("Get filtered target")
            # 平滑化を行う
            target_diff, target_filtered_base = \
                self.try_filter(target_delayed, state, config.filter_kind)
            target_filtered = target_filtered_base + target_diff

            sw.lap("2nd speed limit")
            # 制御値が速度制限されたものであることを保証する
            target_diff, max_ratio, accel_max_ratio = \
                self.try_second_speed_limit(
                    target_diff,
                    dt,
                    self.last_control_velocity,
                    config.use_second_speed_limit,
                    config.filter_kind,
                )

            sw.lap("Get control")
            # 制御値を生成する
            control = target_filtered_base + target_diff
            # 制御値を更新する
            self.control = control
            self.shm.joint_control = control
            # NOTE: "original"を保存する価値がなければ削除可能
            if config.filter_kind == "original":
                self._filter.filter(control)

            # 分析用データ保存
            sw.lap("Save control - gather data")
            datum = [
                dict(
                    time=now,
                    kind="target",
                    joint=target_raw.tolist(),
                ),
                dict(
                    time=now,
                    kind="target_delayed",
                    joint=target_delayed_raw.tolist(),
                ),
                dict(
                    time=now,
                    kind="first_speed_limited_target_delayed",
                    joint=target_delayed.tolist(),
                ),
                dict(
                    time=now,
                    kind="target_filtered",
                    joint=target_filtered.tolist(),
                ),
                dict(
                    time=now,
                    kind="control",
                    joint=control.tolist(),
                    max_ratio=max_ratio,
                    accel_max_ratio=accel_max_ratio,
                    first_max_ratio=first_max_ratio,
                    first_accel_max_ratio=first_accel_max_ratio,
                ),
            ]
            sw.lap("Save control - save to queue")
            self.control_to_archiver_queue.put(datum)

            sw.lap("Check elapsed before command")
            # 制御値をロボットに送る前までの処理で時間がかかっていないか確認する
            t_elapsed = time.time() - now
            if t_elapsed > config.t_intv * 2:
                self.logger.warning(
                    f"Control loop is 2 times as slow as expected before command: "
                    f"{t_elapsed} seconds")
                self.logger.warning(sw.summary())

            # ロボットに制御値を送り制御する
            if config.move:
                sw.lap("Send arm command")
                if config.control_interface == "position":
                    success = self.move_joint_servo(
                        control.tolist(), lock, error_info, error_event, stop_event)
                elif config.control_interface == "velocity":
                    v_control = (control - self.last_control) / (now - self.last)
                    success = self.move_joint_servo_by_vel(
                        v_control.tolist(), lock, error_info, error_event, stop_event)
                if not success:
                    break

            sw.lap("Wait control loop")
            t_elapsed = time.time() - now
            t_wait = config.t_intv - t_elapsed
            if t_wait > 0:
                if self.should_wait_control_loop():
                    time.sleep(t_wait)

            sw.lap("Check elapsed after command")
            t_elapsed = time.time() - now
            if t_elapsed > config.t_intv * 2:
                self.logger.warning(
                    f"Control loop is 2 times as slow as expected after command: "
                    f"{t_elapsed} seconds")
                self.logger.warning(sw.summary())

            sw.stop()

            # ユーザーが停止を要求した場合に、ロボットがゆるやかに静止を完了していれば抜ける
            # NOTE: 判定用にインスタンス変数かしている
            if stop:
                if self.is_ready_to_stop():
                    break

            self.last_control = control
            self.last_control_velocity = target_diff / dt
            self.last = now

        # ツールチェンジなどの後でリアルタイム制御が可能になったタイミングを
        # 知らせるためのフラグをリセットしておく
        self.shm.is_controllable = 0

        hand_thread.join()
        if error_event.is_set():
            # TODO: これで例外発生元のスタックトレースが取得できればこれで十分
            raise error_info['exception']
        return True

    def control_loop_w_recover_automatic(self) -> bool:
        """自動復帰を含むリアルタイム制御ループ"""
        self.logger.info("Start Control Loop with Automatic Recover")
        # 自動復帰ループ
        while True:
            try:
                # 制御ループ
                # スレーブモードに入る
                self.enter_servo_mode()
                # 停止するのは、ユーザーが要求した場合か、自然に内部エラーが発生した場合
                self.control_loop()
                # スレーブモードを解除する
                # NOTE: Cobotta Proではスレーブモード解除可能な状態になったら
                # 即時に解除しないと指令値生成遅延になる
                self.leave_servo_mode()       
                # ここまで正常に終了した場合、ユーザーが要求した場合が成功を意味する
                if self.shm.stop_realtime_control == 1:
                    self.shm.stop_realtime_control = 0
                    self.logger.info("User required stop and succeeded")
                    return True
            except Exception as e:
                # 自然に内部エラーが発生した場合、自動復帰を試みる
                # 自動復帰の前にエラーを確実にモニタするため待機
                time.sleep(1)
                self.logger.error("Error in control loop")
                self.logger.error(f"{self.format_error(e)}")

                # 目標値が状態値から大きく離れた場合は自動復帰しない
                if str(e).startswith("Target and state are too different."):
                    self.shm.stop_realtime_control = 0
                    return False

                # 必ずスレーブモードから抜ける
                try:
                    self.leave_servo_mode()
                except Exception as e_leave:
                    self.logger.error("Error leaving servo mode")
                    self.logger.error(f"{self.format_error(e_leave)}")
                    # タイムアウトの場合はスレーブモードは切れているので
                    # 共有メモリを更新する
                    if self.should_recover_automatic_on_timeout_error(e_leave):
                        self.shm.maybe_slave_mode = 0
                    # それ以外は原因不明なのでループは抜ける
                    else:
                        self.shm.stop_realtime_control = 0
                        return False

                # 非常停止ボタンの状態値を最新にするまで待つ必要がある
                time.sleep(1)
                # 非常停止ボタンが押された場合は自動復帰しない
                if self.shm.is_emergency_stopped == 1:
                    self.logger.error("Emergency stop is pressed")
                    self.shm.stop_realtime_control = 0
                    return False

                # タイムアウトの場合は接続からやり直す
                if self.should_recover_automatic_on_timeout_error(e):
                    is_success = self.recover_automatic_on_timeout_error()
                    if not is_success:
                        self.shm.stop_realtime_control = 0
                        return False
                # ここまでに接続ができている場合
                is_success = self.recover_automatic_on_recoverable_error()
                if not is_success:
                    self.shm.stop_realtime_control = 0
                    return False

    def mqtt_control_loop(self) -> None:
        """MQTTによる制御ループ"""
        self.logger.info("Start MQTT Control Loop")
        while True:
            # 停止するのは、ユーザーが要求した場合か、自然に内部エラーが発生した場合
            success_stop = self.control_loop_w_recover_automatic()
            # 停止フラグが成功の場合は、ユーザーが要求した場合のみありうる
            next_tool_id = self.shm.tool_change
            put_down_box = self.shm.demo_put_down_box
            line_cut = self.shm.line_cut
            if success_stop:
                # ツールチェンジが要求された場合
                if next_tool_id != 0:
                    self.logger.info(
                        f"User required tool change to: {next_tool_id}")
                    ret = self.tool_change(next_tool_id)
                    # ツールチェンジに成功した場合は、ループを継続し
                    # 失敗した場合は、ループを抜ける
                    if not ret:
                        break
                # 棚の上の箱を置くことが要求された場合
                elif put_down_box != 0:
                    self.logger.info("User required put down box")
                    # 成功しても失敗してもループを継続する (ツールを変えることによる
                    # 予測できないエラーは起こらないため)
                    _ = self.demo_put_down_box()
                elif line_cut != 0:
                    self.logger.info("User required line cut")
                    # 成功しても失敗してもループを継続する (ツールを変えることによる
                    # 予測できないエラーは起こらないため)
                    _ = self.line_cut()
                # 単なる停止が要求された場合は、ループを抜ける
                else:
                    break
            # 停止フラグが失敗の場合は、ユーザーが要求した場合か、
            # 自然に内部エラーが発生した場合
            else:
                # ツールチェンジが要求された場合
                if next_tool_id != 0:
                    # 要求コマンドのみリセット
                    self.shm.tool_change_result = 2
                    self.shm.tool_change = 0
                # 棚の上の箱を置くことが要求された場合
                elif put_down_box != 0:
                    # 要求コマンドのみリセット
                    self.shm.demo_put_down_box_result = 2
                    self.shm.demo_put_down_box = 0
                elif line_cut != 0:
                    # 要求コマンドのみリセット
                    self.shm.line_cut_result = 2
                    self.shm.line_cut = 0
                # ループを抜ける
                break

    def get_tool_info(
        self, tool_infos: List[Dict[str, Any]], tool_id: int,
    ) -> Dict[str, Any]:
        return [tool_info for tool_info in tool_infos
                if tool_info["id"] == tool_id][0]

    def find_and_setup_hand(self, tool_id) -> None:
        # NOTE: 変更する可能性あり
        tool_info = self.get_tool_info(tool_infos, tool_id)
        name = tool_info["name"]
        args = tool_info.get("args", {})
        hand = tool_classes[name](**args)
        if tool_id != -1:
            try:
                hand.connect_and_setup()
            except Exception as e:
                self.logger.error(f"Error connecting to hand: {name}")
                self.logger.error(f"{self.format_error(e)}")
                hand = None
        else:
            hand = None
        self.hand_name = name
        self.hand = hand
        self.tool_id = tool_id
        self.shm.tool_id = tool_id

    def release_hand(self) -> bool:
        self.logger.info("Release hand")
        if self.hand is not None:
            is_success, msg = self.hand.release()
        else:
            is_success = False
            msg = "Hand is not connected"
        if not is_success:
            self.logger.error(f"Error releasing hand: {msg}")
        return is_success

    def get_hand_state(
        self,
        lock,
        error_info,
        error_event,
        stop_event,
    ) -> bool:
        # ハンドの状態値を取得して共有メモリに格納する
        if self.hand is not None:
            width, width_success, width_msg = self.hand.get_width()
        else:
            width, width_success, width_msg = None, False, "Hand is not connected"
        if width is None:
            width = 0
        else:
            # 0に意味があるのでオフセットをもたせる
            width += 100
        self.shm.hand_state = width
        if not width_success:
            self.logger.error(f"Failed to get hand state width: {width_msg}")
            with lock:
                error_info['kind'] = "hand"
                error_info['msg'] = width_msg
                error_info['exception'] = ValueError(width_msg)
            error_event.set()
            stop_event.set()

        if self.hand is not None:
            force, force_success, force_msg = self.hand.get_force()
        else:
            force, force_success, force_msg = None, False, "Hand is not connected"
        if force is None:
            force = 0
        else:
            # 0に意味があるのでオフセットをもたせる
            force += 100
        self.shm.hand_force = force
        if not force_success:
            with lock:
                error_info['kind'] = "hand"
                error_info['msg'] = force_msg
                error_info['exception'] = ValueError(force_msg)
            error_event.set()
            stop_event.set()

        return width_success and force_success

    def send_grip(
        self,
        lock,
        error_info,
        error_event,
        stop_event,
    ) -> bool:
        if self.hand is not None:
            is_success, msg = self.hand.grip()
        else:
            is_success = False
            msg = "Hand is not connected"
        if not is_success:
            self.logger.error(f"Failed to grip: {msg}")
            with lock:
                error_info['kind'] = "hand"
                error_info['msg'] = msg
                error_info['exception'] = ValueError(msg)
            error_event.set()
            stop_event.set()
        return is_success

    def send_release(
        self,
        lock,
        error_info,
        error_event,
        stop_event,
    ) -> bool:
        if self.hand is not None:
            is_success, msg = self.hand.release()
        else:
            is_success = False
            msg = "Hand is not connected"
        if not is_success:
            self.logger.error(f"Failed to release: {msg}")
            with lock:
                error_info['kind'] = "hand"
                error_info['msg'] = msg
                error_info['exception'] = ValueError(msg)
            error_event.set()
            stop_event.set()
        return is_success

    def tool_change(self, next_tool_id: int) -> bool:
        """ツールを切り替える。"""
        self.logger.info(f"Tool change to: {next_tool_id}")
        try:
            if next_tool_id == self.tool_id:
                self.logger.info("Selected tool is current tool.")
                self.pose[18] = 1
                self.pose[17] = 0
                return True
            ret = self.hw._tool_change_impl(next_tool_id)
            if not ret:
                raise ValueError("Tool change implementation failed")
            # NOTE: より良い方法がないか
            # VRアニメーションがロボットの動きに追従し終わるのを待つ
            time.sleep(3)
            self.pose[18] = 1
            self.pose[17] = 0
            # VRのIKで解いた関節角度にロボットの関節角度を合わせるのを待つ
            time.sleep(3)
            self.logger.info("Tool change succeeded")
            return True
        except Exception as e:
            self.logger.error("Error during tool change")
            self.logger.error(f"{self.hw.format_error(e)}")
            self.pose[18] = 2
            self.pose[17] = 0
            return False

    def demo_put_down_box(self) -> bool:
        """箱を動かすデモ。"""
        self.logger.info("Demo put down box")
        try:
            ret = self.hw._demo_put_down_box_impl()
            if not ret:
                raise ValueError("Demo put down box implementation failed")
            return True
        except Exception as e:
            self.logger.error("Error during demo put down box")
            self.logger.error(f"{self.hw.format_error(e)}")
            return False

    def line_cut(self) -> bool:
        """箱を切るデモ。"""
        self.logger.info("Line cut")
        try:
            ret = self.hw._line_cut_impl()
            if not ret:
                raise ValueError("Line cut implementation failed")
            return True
        except Exception as e:
            self.logger.error("Error during line cut")
            self.logger.error(f"{self.hw.format_error(e)}")
            return False

    def setup_logger(self, log_queue):
        self.logger = logging.getLogger("CTRL")
        if log_queue is not None:
            self.handler = logging.handlers.QueueHandler(log_queue)
        else:
            self.handler = logging.StreamHandler()
        self.logger.addHandler(self.handler)
        self.logger.setLevel(logging.INFO)
        self.robot_logger = logging.getLogger("CTRL-ROBOT")
        if log_queue is not None:
            self.robot_handler = logging.handlers.QueueHandler(log_queue)
        else:
            self.robot_handler = logging.StreamHandler()
        self.robot_logger.addHandler(self.robot_handler)
        self.robot_logger.setLevel(logging.INFO)

    def start_mqtt_control(self) -> bool:
        self.logger.info("Start MQTT control")
        self.shm.is_mqtt_control = 1
        return True

    def stop_mqtt_control(self) -> bool:
        self.logger.info("Stop MQTT control")
        self.shm.stop_realtime_control = 1
        while self.shm.is_mqtt_control != 0:
            time.sleep(0.1)
        return True

    def receive_command_loop(self) -> None:
        """コマンドはサブスレッドで受付、簡単のためMQTTリアルタイム制御以外もサブスレッドで行う"""
        while True:
            if self.control_pipe.poll(timeout=1):
                command_dict = self.control_pipe.recv()
                status = False
                message = ""
                # NOTE: 現在未使用
                result = {}
                command = command_dict["command"]
                params = command_dict.get("params", {})
                wait = command_dict.get("wait", False)
                # MQTTリアルタイム制御中
                if self.shm.is_mqtt_control == 1:
                    if command == "stop_mqtt_control":
                        status = self.stop_mqtt_control()
                    # 他のコマンドは受け付けず失敗をすぐに返す
                    else:
                        message = "MQTT control in progress. Consider stopping MQTT control first."
                    if wait:
                        self.control_pipe.send({"command": command, "status": status, "message": message, "result": result})
                else:
                    # MQTTリアルタイム制御外
                    if command == "enable":
                        status = self.enable()
                    elif command == "disable":
                        status = self.disable()
                    elif command == "set_area_enabled":
                        status = self.set_area_enabled(**params)
                    elif command == "tidy_pose":
                        status = self.tidy_pose()
                    elif command == "release_hand":
                        status = self.release_hand()
                    elif command == "line_cut":
                        status = self.line_cut()
                    elif command == "clear_error":
                        status = self.clear_error()
                    elif command == "start_mqtt_control":
                        status = self.start_mqtt_control()
                    elif command == "tool_change":
                        status = self.tool_change(**params)
                    elif command == "jog_joint":
                        status = self.jog_joint(**params)
                    elif command == "jog_tcp":
                        status = self.jog_tcp(**params)
                    elif command == "move_joint":
                        status = self.move_joint(**params)
                    elif command == "demo_put_down_box":
                        status = self.demo_put_down_box()                
                    else:
                        message = "MQTT control not in progress. Consider starting MQTT control first."
                    if wait:
                        self.control_pipe.send({"command": command, "status": status, "message": message, "result": result})

    def init_receive_command_loop(self):
        self.receive_command_thread = threading.Thread(
            target=self.receive_command_loop)
        self.receive_command_thread.start()
    
    def del_receive_command_loop(self):
        if hasattr(self, 'receive_command_thread'):
            self.receive_command_thread.join()

    def run_proc(self, control_pipe, slave_mode_lock, log_queue, control_to_archiver_queue, monitor_queue):
        self.setup_logger(log_queue)
        self.logger.info("Process started")
        self.shm = NamedSharedMemory(create=False)
        self.slave_mode_lock = slave_mode_lock
        self.control_pipe = control_pipe
        self.control_to_archiver_queue = control_to_archiver_queue
        self.monitor_queue = monitor_queue

        self.init_realtime()
        # NOTE: connect_robotはenabledなどと同様boolを返すようにしているが
        # 今後変更する可能性あり
        is_connected = self.connect_robot()
        if is_connected:
            # リアルタイム性を考慮し、MQTTリアルタイム制御はメインスレッドで行う
            # コマンドはサブスレッドで受付、簡単のためMQTTリアルタイム制御以外もサブスレッドで行う
            self.init_receive_command_loop()
            # 外のループでMQTTリアルタイム制御の開始とプロセス終了を監視する
            while True:
                if self.shm.is_mqtt_control == 1:
                    # 内のループでMQTTリアルタイム制御を行う
                    self.mqtt_control_loop()
                    self.shm.is_mqtt_control = 0
                if self.shm.exit_program == 1:                
                    break
                # 監視間隔はリアルタイムでなくて良い
                time.sleep(0.1)
        self.del_robot()
        self.del_monitor_loop()
        self.del_receive_command_loop()
        self.logger.info("Clean up threads")
        # 他のプロセスにも周知
        self.shm.exit_program = 1
        time.sleep(1)
        self.shm.release()
        self.control_to_archiver_queue.close()
        self.monitor_queue.close()
        self.logger.info("Shared memory released")
        self.logger.info("Process stopped")
        time.sleep(1)
        self.handler.close()
        self.robot_handler.close()
