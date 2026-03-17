import logging
import os
import sys
import threading
import time
from abc import ABC, abstractmethod
from typing import Any, Dict, List, TextIO

import modern_robotics as mr
import numpy as np
import psutil

from ..common.filter import SMAFilter
from ..common.interpolate import DelayedInterpolator
from ..common.utils import StopWatch
from ..robot.config import (
    N_JOINTS,
    T_INTV,
    abs_joint_soft_limit,
    control_interface,
    delay_for_interpolation,
    eff_accel_limits,
    eff_speed_limits,
    filter_kind,
    move_robot,
    n_windows,
    stopped_velocity_eps,
    target_state_abs_joint_diff_limit,
    use_first_speed_limit,
    use_interp,
    use_normalize_target_to_nearest,
    use_second_speed_limit,
)
from .shared_memory import NamedSharedMemory


class ControlBase(ABC):
    """ロボットの制御ループの基底クラス."""

    # START: 実装必須

    @abstractmethod
    def init_robot(self) -> None:
        pass

    @abstractmethod
    def robot_log_loop_step(self) -> None:
        pass

    @abstractmethod
    def real_to_vr_joint(self, joints: List[float]) -> List[float]:
        pass

    @abstractmethod
    def get_current_pose_rt(self) -> List[float]:
        pass

    @abstractmethod
    def get_current_joint_rt(self) -> List[float]:
        pass

    @abstractmethod
    def get_current_force_rt(self) -> List[float]:
        pass

    @abstractmethod
    def get_all_robot_state_at_once(self) -> None:
        pass

    @abstractmethod
    def get_enabled(self) -> bool:
        pass

    @abstractmethod
    def get_is_in_servo_mode(self) -> bool:
        pass

    @abstractmethod
    def get_is_emergency_stopped(self) -> bool:
        pass

    @abstractmethod
    def get_errors(self) -> List[Dict[str, Any]]:
        pass

    @abstractmethod
    def format_error(self, e: Exception) -> str:
        pass

    @abstractmethod
    def move_joint_servo(
        self,
        control: List[float],
        lock,
        error_info,
        error_event,
        stop_event,
    ) -> bool:
        pass

    @abstractmethod
    def move_joint_servo_by_vel(
        self,
        control: List[float],
        lock,
        error_info,
        error_event,
        stop_event,
    ) -> bool:
        pass

    @abstractmethod
    def send_grip(self) -> None:
        pass

    @abstractmethod
    def send_release(self) -> None:
        pass

    @abstractmethod
    def release_hand(self) -> bool:
        pass

    @abstractmethod
    def enable(self) -> bool:
        pass

    @abstractmethod
    def disable(self) -> bool:
        pass

    @abstractmethod
    def set_area_enabled(self, enable: bool) -> bool:
        pass

    @abstractmethod
    def tidy_pose(self) -> bool:
        pass

    @abstractmethod
    def move_joint(self, joints: List[float]) -> bool:
        pass

    @abstractmethod
    def clear_error(self) -> bool:
        pass

    @abstractmethod
    def enter_servo_mode(self) -> bool:
        pass

    @abstractmethod
    def leave_servo_mode(self) -> bool:
        pass

    @abstractmethod
    def should_recover_automatic_on_timeout_error(self, e_leave) -> bool:
        pass

    @abstractmethod
    def recover_automatic_on_timeout_error(self) -> bool:
        pass

    @abstractmethod
    def recover_automatic_on_recoverable_error(self) -> bool:
        pass

    @abstractmethod
    def _tool_change_impl(self, next_tool_id: int) -> None:
        pass

    @abstractmethod
    def jog_joint(self, joint: int, direction: float) -> bool:
        pass

    @abstractmethod
    def jog_tcp(self, axis: int, direction: float) -> bool:
        pass

    @abstractmethod
    def demo_put_down_box(self) -> bool:
        pass

    @abstractmethod
    def line_cut(self) -> bool:
        pass

    @abstractmethod
    def del_robot(self) -> None:
        pass

    # STOP: 実装必須

    # START: オーバーライドの可能性あり

    def on_step_start_in_control_loop(self) -> None:
        # ロボット固有の処理を含む
        pass

    def should_wait_control_loop(self) -> bool:
        # ロボット固有の処理を含む
        return True

    def is_ready_to_stop(self) -> bool:
        # ロボット固有の処理を含む
        # スレーブモードでは十分低速時に2回同じ位置のコマンドを送ると
        # ロボットを停止させてスレーブモードを解除可能な状態になる
        # TODO: Cobottaには必要だがURに必要かは不明
        return (self.control == self.last_control).all()

    # STOP: オーバーライドの可能性あり

    # START: オーバーライドの可能性なし

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

    def init_robot_log_loop(self) -> None:
        self.robot_log_thread = threading.Thread(
            target=self.robot_log_loop)
        self.robot_log_thread.start()

    def del_robot_log(self) -> None:
        if hasattr(self, 'robot_log_thread'):
            self.robot_log_thread.join()

    def robot_log_loop(self) -> None:
        while True:
            now = time.time()
            self.robot_log_loop_step()
            if self.shm.exit_program == 1:
                break
            t_elapsed = time.time() - now
            # ログの優先度は低いため周期を長くする
            t_wait = T_INTV * 2 - t_elapsed
            if t_wait > 0:
                time.sleep(t_wait)

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
            t_wait = T_INTV - t_elapsed
            if t_wait > 0:
                time.sleep(t_wait)

    def hand_control_loop(
        self, stop_event, error_event, lock, error_info,
    ) -> None:
        self.logger.info("Start Hand Control Loop")
        last_tool_corrected = None
        t_intv_hand = T_INTV * 2
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
                try:
                    if tool_corrected == 1:
                        self.logger.info("Send grip command to hand")
                        self.send_grip()
                    elif tool_corrected == 2:
                        self.logger.info("Send release command to hand")
                        self.send_release()
                except Exception as e:
                    with lock:
                        error_info['kind'] = "hand"
                        error_info['msg'] = self.format_error(e)
                        error_info['exception'] = e
                    error_event.set()
                    break
            # ハンドの状態値を取得
            # 情報を常に取得するとアームの制御ループの処理間隔を乱し
            # 情報が必要なのはハンドに制御値を送った少し後だけなので以下のようにする
            # NOTE: 常に取得した方がいいかもしれない。処理間隔を乱すなら別プロセス化の方がいいかもしれない
            if now - last_tool_corrected_time < 1:
                try:
                    self.get_hand_state()
                except Exception as e:
                    with lock:
                        error_info['kind'] = "hand"
                        error_info['msg'] = self.format_error(e)
                        error_info['exception'] = e
                    error_event.set()
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
                eff_speed_limits, eff_accel_limits,
            )
            target_diff = v * dt

        # 速度がしきい値より小さければ静止させ、ドリフトや振動を避ける
        # NOTE: どのロボットにも有意義な処理である。特にCobotta Proの
        # スレーブモードを正常に解除するためにも必要
        if np.all(v < stopped_velocity_eps):
            target_diff = np.zeros(N_JOINTS)

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
                eff_speed_limits, eff_accel_limits,
            )
            target_diff = v * dt

        # 速度がしきい値より小さければ静止させ、ドリフトや振動を避ける
        # NOTE: どのロボットにも有意義な処理である。特にCobotta Proの
        # スレーブモードを正常に解除するためにも必要
        if np.all(v < stopped_velocity_eps):
            target_diff = np.zeros(N_JOINTS)

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
            N = N_JOINTS
            Tf = T_INTV * (N - 1)
            method = 5
            Kp = 0.6
            Kd = 0.02
            prev_error = np.zeros(N_JOINTS)
            pd_step = 0
            self._filter_params = {
                "N": N,
                "Tf": Tf,
                "method": method,
                "Kp": Kp,
                "Kd": Kd,
                "prev_error": prev_error,
                "pd_step": pd_step,
                "last_control_velocity": np.zeros((N - 1, N_JOINTS)),
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
                dt = T_INTV
                # [N - 1, N_JOINTS]
                target_diffs = np.diff(target_steps, axis=0)
                vs = target_diffs / dt
                ratios = np.abs(vs) / eff_speed_limits[None, :]
                max_ratio = np.max(ratios)
                if max_ratio > 1:
                    vs /= max_ratio

                # 加速度制限
                # [N, N_JOINTS]
                vs_ = np.concatenate([last_control_velocity[[-1], :], vs], axis=0)
                # [N - 1, N_JOINTS]
                as_ = np.diff(vs_, axis=0) / dt
                accel_ratios = np.abs(as_) / eff_accel_limits[None, :]
                accel_max_ratio = np.max(accel_ratios)
                if accel_max_ratio > 1:
                    as_ /= accel_max_ratio
                # [N - 1, N_JOINTS]
                vs_ = vs_[0][None, :] + np.cumsum(as_, axis=0) * dt

                target_diffs_speed_limited = vs_ * dt
                # 速度がしきい値より小さければ静止させ、ドリフトや振動を避ける
                # NOTE: どのロボットにも有意義な処理である。特にCobotta Proの
                # スレーブモードを正常に解除するためにも必要
                for i in range(N - 1):
                    if np.all(target_diffs_speed_limited[i] / dt < stopped_velocity_eps):
                        target_diffs_speed_limited[i] = np.zeros_like(
                            target_diffs_speed_limited[i])
                        vs_[i] = target_diffs_speed_limited[i] / dt

                # [N - 1, N_JOINTS]
                target_steps_speed_limited = target_steps[0][None, :] + np.cumsum(vs_, axis=0) * dt
                last_control_velocity = vs_
                self._filter_params["last_control_velocity"] = last_control_velocity

            target_filtered = target_steps_speed_limited[pd_step]
            target_filtered_base = target_filtered
            target_diff = np.zeros(N_JOINTS)

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

    def control_loop(self, f: TextIO | None = None) -> bool:
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
                time.sleep(T_INTV)
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
                time.sleep(T_INTV)
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
            if use_normalize_target_to_nearest:
                target_norm = state + (target - state + 180) % 360 - 180
                target = target_norm

            # TODO: VR側でもソフトリミットを設定したほうが良い
            target_th = np.maximum(target, -abs_joint_soft_limit)
            if (target != target_th).any():
                self.logger.warning("target reached minimum threshold")
            target_th = np.minimum(target_th, abs_joint_soft_limit)
            if (target != target_th).any():
                self.logger.warning("target reached maximum threshold")
            target = target_th

            # 目標値が状態値から大きく離れた場合
            if (np.abs(target - state) > 
                target_state_abs_joint_diff_limit).any():
                # NOTE: まだ目標値を受け取っていない場合か、
                # すでに目標値を受け取っていて前回の目標値からも大きく離れていた場合のみ、
                # 停止させる
                # それ以外の場合は、目標値に比べて制御値、状態値の追従が遅れているだけで
                # ありえるので停止させない
                # VR側との同期方法が改善されれば不要な処理になる可能性あり
                if (
                    last_target is None or
                    (np.abs(target - last_target) > 
                    target_state_abs_joint_diff_limit).any()
                ):
                    # 強制停止する。ゆるやかな停止ではエラーの返し方が複雑になるため
                    msg = "Target and state are too different.\n"
                    msg += "| Joint  | Target | State  | Diff   | Limit  |\n"
                    msg += "| ------ | ------ | ------ | ------ | ------ |\n"
                    for i in range(N_JOINTS):
                        if abs(target[i] - state[i]) > target_state_abs_joint_diff_limit[i]:
                            msg += f"| {i: <6d} | {target[i]: <6.1f} | {state[i]: <6.1f} | {abs(target[i] - state[i]): <6.1f} | {target_state_abs_joint_diff_limit[i]: <6.1f} |\n"
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
                if use_interp:
                    di = DelayedInterpolator(delay=delay_for_interpolation)
                    di.reset(now, target)
                    target_delayed = di.read(now, target)
                else:
                    target_delayed = target


                self.last_target_delayed = target_delayed
                self.last_target_delayed_velocity = np.zeros(N_JOINTS)

                # 制御値の初期化
                self.last_control = state
                self.last_control_velocity = np.zeros(N_JOINTS)

                # 移動平均フィルタのセットアップ
                self.init_filter(filter_kind, n_windows, state, target)

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
            if use_interp:
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
                    use_first_speed_limit,
                )
            target_delayed = self.last_target_delayed + target_diff
            self.last_target_delayed = target_delayed
            self.last_target_delayed_velocity = target_diff / dt

            sw.lap("Get filtered target")
            # 平滑化を行う
            target_diff, target_filtered_base = \
                self.try_filter(target_delayed, state, filter_kind)
            target_filtered = target_filtered_base + target_diff

            sw.lap("2nd speed limit")
            # 制御値が速度制限されたものであることを保証する
            target_diff, max_ratio, accel_max_ratio = \
                self.try_second_speed_limit(
                    target_diff,
                    dt,
                    self.last_control_velocity,
                    use_second_speed_limit,
                    filter_kind,
                )

            sw.lap("Get control")
            # 制御値を生成する
            control = target_filtered_base + target_diff
            # 制御値を更新する
            self.control = control
            self.shm.joint_control = control
            # NOTE: "original"を保存する価値がなければ削除可能
            if filter_kind == "original":
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
            if t_elapsed > T_INTV * 2:
                self.logger.warning(
                    f"Control loop is 2 times as slow as expected before command: "
                    f"{t_elapsed} seconds")
                self.logger.warning(sw.summary())

            # ロボットに制御値を送り制御する
            if move_robot:
                sw.lap("Send arm command")
                if control_interface == "position":
                    success = self.move_joint_servo(
                        control.tolist(), lock, error_info, error_event, stop_event)
                elif control_interface == "velocity":
                    v_control = (control - self.last_control) / (now - self.last)
                    success = self.move_joint_servo_by_vel(
                        v_control.tolist(), lock, error_info, error_event, stop_event)
                if not success:
                    break

            sw.lap("Wait control loop")
            t_elapsed = time.time() - now
            t_wait = T_INTV - t_elapsed
            if t_wait > 0:
                if self.should_wait_control_loop():
                    time.sleep(t_wait)

            sw.lap("Check elapsed after command")
            t_elapsed = time.time() - now
            if t_elapsed > T_INTV * 2:
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

    def tool_change(self, next_tool_id: int) -> bool:
        try:
            if next_tool_id == self.tool_id:
                self.logger.info("Selected tool is current tool.")
                self.pose[18] = 1
                self.pose[17] = 0
                return True
            self._tool_change_impl(next_tool_id)
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
            self.logger.error(f"{self.robot.format_error(e)}")
            self.pose[18] = 2
            self.pose[17] = 0
            return False

    def tool_change_not_in_rt(self, tool_id: int) -> bool:
        self.logger.info("Tool change not in real-time")
        while True:
            next_tool_id = self.pose[17]
            if next_tool_id != 0:
                self.pose[41] = 0
                ret = self.tool_change(next_tool_id)
                self.pose[41] = 1
                return ret

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
                        status = self.tool_change_not_in_rt(**params)
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

        self.init_robot()
        self.init_realtime()
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
                self.del_robot()
                self.del_robot_log()
                self.del_monitor_loop()
                self.del_receive_command_loop()
                self.shm.release()
                self.control_to_archiver_queue.close()
                self.monitor_queue.close()
                time.sleep(1)
                self.logger.info("Process stopped")
                self.handler.close()
                self.robot_handler.close()
                break
            # 監視間隔はリアルタイムでなくて良い
            time.sleep(0.1)

    # STOP: オーバーライドの可能性なし
