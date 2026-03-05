# Doosanを制御する

import logging
import queue
from typing import Any, Dict, List, Literal, TextIO, Tuple
import datetime
import time
import traceback

import os
import sys
import json
import psutil

import threading

import modern_robotics as mr
import numpy as np
from dotenv import load_dotenv

# Robot shared modules
from ..common.filter import SMAFilter
from ..common.interpolate import DelayedInterpolator
from ..common.utils import deg2rad_list, StopWatch

# Robot specific modules
from .config import ABS_JOINT_LIMIT, N_JOINTS, T_INTV
from .shared_memory import NamedSharedMemory
from .doosan_robot import DoosanRobot, ROBOT_STATE
from .tools import tool_infos, tool_classes, tool_base
from .qbsofthand_industry_api_pybind import qbSoftHandIndustryAPI


# パラメータ
load_dotenv(os.path.join(os.path.dirname(__file__),'.env'))
SAVE = os.getenv("SAVE", "true") == "true"
MOVE = os.getenv("MOVE", "true") == "true"
# Simulated robot
# robot_ip = "127.0.0.1"
# Real robot
# robot_ip = "10.5.5.102"
ROBOT_IP = os.getenv("ROBOT_IP", "10.5.5.102")
HAND_IP = os.getenv("HAND_IP", "192.168.5.44")

# 基本的に運用時には固定するパラメータ
# 実際にロボットを制御するかしないか (VRとの結合時のデバッグ用)
move_robot = MOVE
# 平滑化の方法
# NOTE: 実際のVRコントローラとの結合時に、
# 遅延などを考慮すると改良が必要かもしれない。
# そのときのヒントとして残している
filter_kind: Literal[
    "original",
    "target",
    "state_and_target_diff",
    "moveit_servo_humble",
    "control_and_target_diff",
    "feedback_pd_traj",
    "none",
    "filter_target_from_target_but_diff_from_control"
] = "filter_target_from_target_but_diff_from_control"  # "original"
speed_limits = np.array([180, 180, 180, 360, 360, 360])
speed_limit_ratio = 0.5
accel_limits = speed_limits ** 2
accel_limit_ratio = 0.5
stopped_velocity_eps = 1e-4
servo_mode = 0x202
use_interp = True
n_windows = 10
if filter_kind == "original":
    n_windows = 10
elif filter_kind == "target":
    n_windows = 100
if servo_mode == 0x102:
    t_intv = 0.004
else:
    t_intv = T_INTV
# n_windows *= int(0.008 / t_intv)
reset_default_state = True
default_joints = {
    # TCPが台の中心の上に来る初期位置
    "tidy": [0.0, 0.0, -90.0, 0.0, -90.0, 0.0],
    # NOTE: j5の基準がVRと実機とでずれているので補正。将来的にはVR側で修正?
    "vr": [159.3784, 10.08485, 122.90902, 151.10866, -43.20116 + 90, 20.69275],
    # NOTE: 2025/04/18 19:25の新しい位置?VRとの対応がおかしい気がする
    # 毎回の値も[0, 0, 0, 0, 0, 0]が飛んでくる気がする
    "vr2": [115.55677, 5.86272, 135.70465, 110.53529, -15.55474 + 90, 35.59977],
    # NOTE: 2025/05/30での新しい位置
    "vr3": [113.748, 5.645, 136.098, 109.059, 75.561, 35.82],
    # NOTE: 2025/06/05 での新しい位置
    "vr4": [-46.243, 10.258, 128.201, 125.629, 62.701, 32.618],
    "vr5": [-66.252, 5.645, 136.098, 109.059, 75.561, 35.82],
}
abs_joint_limit = ABS_JOINT_LIMIT
abs_joint_limit = np.array(abs_joint_limit)
abs_joint_soft_limit = abs_joint_limit - 10
# 外部速度。単位は%
speed_normal = 20
speed_tool_change = 2
# 目標値が状態値よりこの制限より大きく乖離した場合はロボットを停止させる
# 設定値は典型的なVRコントローラの動きから決定した
target_state_abs_joint_diff_limit = [30, 30, 40, 40, 40, 60]
use_first_speed_limit = True
use_second_speed_limit = True
control_interface: Literal["position", "velocity"] = "velocity"
save_control = SAVE
use_normalize_target_to_nearest = True

class Doosan_CON:
    def __init__(self):
        self.default_joint = default_joints["vr5"]
        self.tidy_joint = default_joints["tidy"]
        self.robot: DoosanRobot | None = None
        self.qb_hand: qbSoftHandIndustryAPI | None = None
        self.all_robot_state = {}

    def init_robot(self):
        # TODO: 要改善
        # ロボット固有の処理を含む
        try:
            use_robot_log_loop = True
            use_monitor_loop = True
            if self.robot is None:
                self.robot = DoosanRobot(ROBOT_IP, "queue", t_intv)
                if not self.robot.start():
                    raise ValueError("Failed to start robot")
                if use_robot_log_loop:
                    self.init_robot_log_loop()                
                if use_monitor_loop:
                    self.init_monitor_loop()
            tool_id = int(os.environ["TOOL_ID"])
            self.find_and_setup_hand(tool_id)
        except Exception as e:
            self.logger.error("Error in initializing robot: ")
            self.logger.error(f"{self.format_error(e)}")

    def init_robot_log_loop(self):
        self.robot_log_thread = threading.Thread(
            target=self.robot_log_loop)
        self.robot_log_thread.start()

    def del_robot_log(self):
        if hasattr(self, 'robot_log_thread'):
            self.robot_log_thread.join()

    def robot_log_loop_step(self):
        # ロボット固有の処理を含む
        log_block = self.robot.pop_log_queue()
        for log in log_block:
            # log is a tuple: (timestamp, level, message)
            timestamp, level, message = log                
            # ログレコードを手動で作成してタイムスタンプを反映
            log_record = logging.LogRecord(
                name=self.robot_logger.name,
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
            if self.robot_logger.isEnabledFor(log_record.levelno):
                self.robot_logger.handle(log_record)

    def robot_log_loop(self):
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

    def init_monitor_loop(self):
        self.monitor_thread = threading.Thread(
            target=self.monitor_loop)
        self.monitor_thread.start()
    
    def del_monitor_loop(self):
        if hasattr(self, 'monitor_thread'):
            self.monitor_thread.join()

    def real_to_vr_joint(self, joints: List[float]) -> List[float]:
        # ロボット固有の処理を含む
        return deg2rad_list(joints)

    def get_current_pose_rt(self) -> List[float]:
        # ロボット固有の処理を含む
        return self.robot.get_current_pose_rt()[1:]

    def get_current_joint_rt(self) -> List[float]:
        # ロボット固有の処理を含む
        return self.robot.get_current_joint_rt()[1:]
    
    def get_current_force_rt(self) -> List[float]:
        # ロボット固有の処理を含む
        return self.robot.get_current_external_tcp_force_rt()[1:]

    def get_all_robot_state_at_once(self) -> None:
        """ロボットの状態値が個別の関数ではなく少数の関数でまとめて取得できる場合に使用"""
        # ロボット固有の処理を含む
        self.all_robot_state["robot_state"] = self.robot.get_robot_state()

    def get_enabled(self) -> bool:
        # ロボット固有の処理を含む
        robot_state = self.all_robot_state["robot_state"]
        return robot_state in [
            ROBOT_STATE.STATE_STANDBY,
            ROBOT_STATE.STATE_MOVING,
            ROBOT_STATE.STATE_TEACHING,
            ROBOT_STATE.STATE_HOMMING,
        ]

    def get_is_in_servo_mode(self) -> bool:
        # ロボット固有の処理を含む
        # Doosanではスレーブモードの状態はAPIでは不明なので制御値を使用
        return bool(self.shm.maybe_slave_mode)
    
    def get_is_emergency_stopped(self) -> bool:
        # ロボット固有の処理を含む
        robot_state = self.all_robot_state["robot_state"]
        return robot_state == ROBOT_STATE.STATE_EMERGENCY_STOP

    def get_errors(self) -> List[Dict[str, Any]]:
        # ロボット固有の処理を含む
        errors = []
        robot_state = self.all_robot_state["robot_state"]
        is_normal_mode = robot_state not in [
            # ROBOT_STATE.STATE_SAFE_OFF,
            ROBOT_STATE.STATE_SAFE_STOP,
            ROBOT_STATE.STATE_SAFE_OFF2,
            ROBOT_STATE.STATE_SAFE_STOP2,
        ]
        if not is_normal_mode:
            errors = [{"error_code": 0,
                       "error_message": "Robot state is not NORMAL"}]
        return errors

    def monitor_loop(self):
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

    def get_hand_state(self):
        # TODO
        # ロボット固有の処理を含む
        # ハンドの状態値を取得して共有メモリに格納する
        width = None
        force = None
        if self.qb_hand is not None:
            width = self.qb_hand.getPosition()
            force = self.qb_hand.getCurrent()
        if width is None:
            width = 0
        else:
            # 0に意味があるのでオフセットをもたせる
            width += 100
        if force is None:
            force = 0
        else:
            force += 100
        self.shm.hand_state = width
        self.shm.hand_force = force

    def find_and_setup_hand(self, tool_id):
        # ダミー処理
        connected = False
        tool_info = self.get_tool_info(tool_infos, tool_id)
        name = tool_info["name"]
        hand = tool_classes[name]()
        if tool_id != -1:
            connected = hand.connect_and_setup()
            if not connected:
                raise ValueError(f"Failed to connect to hand: {name}")
        else:
            hand = None
        self.hand_name = name
        self.hand = hand
        self.tool_id = tool_id
        self.shm.tool_id = tool_id
        # NOTE: ツールチェンジ時は同等の機能の追加が必要
        # if tool_id != -1:
        #     self.robot.SetToolDef(
        #         tool_info["id_in_robot"], tool_info["tool_def"])
        # self.robot.set_tool(tool_info["id_in_robot"])
        # 本処理
        max_timeout = 10  # seconds
        self.qb_hand = qbSoftHandIndustryAPI(HAND_IP, max_timeout)
        if not self.qb_hand.isInitialized():
            raise ValueError("Failed to initialize qbSoftHandIndustryAPI")

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

    def format_error(self, e: Exception) -> str:
        # ロボット固有の処理を含む
        s = "\n"
        s = s + "Error trace: " + traceback.format_exc() + "\n"
        return s

    def hand_control_loop(self, stop_event, error_event, lock, error_info):
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

    def control_loop(self, f: TextIO | None = None) -> bool:
        """リアルタイム制御ループ"""
        # ロボット固有の処理を含まない
        self.enter_servo_mode()
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

        while True:
            sw.start("Get shared memory")
            now = time.time()
            self.on_step_start_in_control_loop()

            # ハンドの制御が止まった場合はアームの制御も止める
            if not hand_thread.is_alive():
                break

            # ユーザーが停止を要求した場合
            # ロボットに制御値を送る前後でアームの制御の止め方が変わる
            stop = self.shm.stop_realtime_control

            # 状態値を取得しているかを確認
            if self.shm.is_joint_state_received != 1:
                if self.stop_by_user_or_emergency_before_sending_control(
                    stop, stop_event, error_event, lock, error_info
                ):
                    break
                # 制御の滑らかさ評価で、予め決められた時間ごとの角度（= 軌道）を
                # targetとして使用する場合、以下で待ちすぎると最初に取得される
                # targetがstateから大きく離れ、ガッとロボットが動くので、短い時間だけ待つ
                time.sleep(t_intv)
                continue

            # ツールチェンジなど後の制御可能フラグ
            self.shm.is_controllable = 1

            # 目標値を取得しているかを確認
            if self.shm.is_joint_target_received != 1:
                if self.stop_by_user_or_emergency_before_sending_control(
                    stop, stop_event, error_event, lock, error_info
                ):
                    break
                # 制御の滑らかさ評価で、予め決められた時間ごとの角度（= 軌道）を
                # targetとして使用する場合、以下で待ちすぎると最初に取得される
                # targetがstateから大きく離れ、ガッとロボットが動くので、短い時間だけ待つ
                time.sleep(t_intv)
                continue

            # 関節の状態値
            state = self.shm.joint_state.copy()
            # 関節の目標値
            target = self.shm.joint_target.copy()

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
            # とりあえず急に動こうとすれば止まる仕組みは入れている
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
            # すでに目標値を受け取っていない場合はすぐに、
            # 目標値を受け取っている場合は前回の目標値からも大きく離れた場合に停止させる
            if (
                (np.abs(target - state) > 
                target_state_abs_joint_diff_limit).any()
            ) and (
                last_target is None or
                (np.abs(target - last_target) > 
                target_state_abs_joint_diff_limit).any()
            ):
                # 強制停止する。強制停止しないとエラーメッセージを返すのが複雑になる
                msg = f"Target and state are too different. State: {state}, Target: {target}, Last target: {last_target}, Diff limit: {target_state_abs_joint_diff_limit}"
                with lock:
                    error_info['kind'] = "robot"
                    error_info['msg'] = msg
                    error_info['exception'] = ValueError(msg)
                error_event.set()
                stop_event.set()
                break

            last_target = target

            sw.lap("First target")
            if self.last == 0:
                self.logger.info("Start sending control command")
                self.last = now

                if self.stop_by_user_or_emergency_before_sending_control(
                    stop, stop_event, error_event, lock, error_info
                ):
                    break

                # 目標値を遅延を許して極力線形補間するためのセットアップ
                if use_interp:
                    di = DelayedInterpolator(delay=0.1)
                    di.reset(now, target)
                    target_delayed = di.read(now, target)
                else:
                    target_delayed = target
                self.last_target_delayed = target_delayed
                
                # 移動平均フィルタのセットアップ（t_intv秒間隔）
                if filter_kind == "original":
                    self.last_control = state
                    self.last_control_velocity = np.zeros(N_JOINTS)
                    self._filter = SMAFilter(n_windows=n_windows)
                    self._filter.reset(state)
                elif filter_kind == "target":
                    self.last_control_velocity = np.zeros(N_JOINTS)
                    self._filter = SMAFilter(n_windows=n_windows)
                    self._filter.reset(target)
                elif filter_kind == "filter_target_from_target_but_diff_from_control":
                    self.last_control = state
                    self.last_control_velocity = np.zeros(N_JOINTS)
                    self._filter = SMAFilter(n_windows=n_windows)
                    self._filter.reset(target)
                elif filter_kind == "state_and_target_diff":
                    self.last_control = state
                    self.last_control_velocity = np.zeros(N_JOINTS)
                    self._filter = SMAFilter(n_windows=n_windows)
                    self._filter.reset(state)
                elif filter_kind == "moveit_servo_humble":
                    self.last_control_velocity = np.zeros(N_JOINTS)
                    self._filter = SMAFilter(n_windows=n_windows)
                    self._filter.reset(state)
                elif filter_kind == "control_and_target_diff":
                    self.last_control = state
                    self.last_control_velocity = np.zeros(N_JOINTS)
                    self._filter = SMAFilter(n_windows=n_windows)
                    self._filter.reset(state)
                elif filter_kind == "feedback_pd_traj":
                    N = N_JOINTS
                    Tf = t_intv * (N - 1)
                    method = 5
                    Kp = 0.6
                    Kd = 0.02
                    prev_error = np.zeros(N_JOINTS)
                    pd_step = 0
                    self.last_control_velocity = np.zeros((N - 1, N_JOINTS))
                elif filter_kind == "none":
                    self.last_control = state
                    self.last_control_velocity = np.zeros(N_JOINTS)

                # 速度制限をフィルタの手前にも入れてみる
                self.last_target_delayed_velocity = np.zeros(N_JOINTS)

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
            # 速度制限をフィルタの手前にも入れてみる
            first_max_ratio = None
            first_accel_max_ratio = None
            if use_first_speed_limit:
                target_diff = target_delayed - self.last_target_delayed
                # 速度制限
                dt = now - self.last
                v = target_diff / dt
                ratio = np.abs(v) / (speed_limit_ratio * speed_limits)
                max_ratio = np.max(ratio)
                if max_ratio > 1:
                    v /= max_ratio
                target_diff_speed_limited = v * dt
                first_max_ratio = max_ratio

                # 加速度制限
                a = (v - self.last_target_delayed_velocity) / dt
                accel_ratio = np.abs(a) / (accel_limit_ratio * accel_limits)
                accel_max_ratio = np.max(accel_ratio)
                if accel_max_ratio > 1:
                    a /= accel_max_ratio
                v = self.last_target_delayed_velocity + a * dt
                target_diff_speed_limited = v * dt
                first_accel_max_ratio = accel_max_ratio

                # 速度がしきい値より小さければ静止させ無駄なドリフトを避ける
                # NOTE: スレーブモードを落とさないためには前の速度が十分小さいとき (しきい値は不明) 
                # にしか静止させてはいけない
                if np.all(target_diff_speed_limited / dt < stopped_velocity_eps):
                    target_diff_speed_limited = np.zeros_like(
                        target_diff_speed_limited)
                    v = target_diff_speed_limited / dt

                self.last_target_delayed_velocity = v
                target_delayed = self.last_target_delayed + target_diff_speed_limited

            self.last_target_delayed = target_delayed

            sw.lap("Get filtered target")
            # 平滑化
            if filter_kind == "original":
                # 成功している方法
                # 速度制限済みの制御値で平滑化をしており、
                # moveit servoなどでは見られない処理
                target_filtered = self._filter.predict_only(target_delayed)
                target_diff = target_filtered - self.last_control
            elif filter_kind == "target":
                # 成功することもあるが平滑化窓を増やす必要あり
                # 状態値を無視した目標値の値をロボットに送る
                last_target_filtered = self._filter.previous_filtered_measurement
                target_filtered = self._filter.filter(target_delayed)
                target_diff = target_filtered - last_target_filtered
            elif filter_kind == "filter_target_from_target_but_diff_from_control":
                target_filtered = self._filter.filter(target_delayed)
                target_diff = target_filtered - self.last_control
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
                target_diff = target_filtered - last_target_filtered
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
                target_diff = target_filtered - state
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
                target_diff = target_filtered - last_target_filtered
            elif filter_kind == "feedback_pd_traj":
                if pd_step == 0:
                    error = target_delayed - state
                    d_error = (error - prev_error) / Tf
                    mse = np.mean(error ** 2)
                    target_goal = state + Kp * error + Kd * d_error
                    prev_error = error
                    target_steps = mr.JointTrajectory(
                        state.tolist(),
                        target_goal.tolist(),
                        Tf,
                        N,
                        method,
                    )
                    # 速度制限
                    dt = t_intv
                    # [N - 1, N_JOINTS]
                    target_diffs = np.diff(target_steps, axis=0)
                    vs = target_diffs / dt
                    ratios = np.abs(vs) / (speed_limit_ratio * speed_limits)[None, :]
                    max_ratio = np.max(ratios)
                    if max_ratio > 1:
                        vs /= max_ratio

                    # 加速度制限
                    # [N, N_JOINTS]
                    vs_ = np.concatenate([self.last_control_velocity[[-1], :], vs], axis=0)
                    # [N - 1, N_JOINTS]
                    as_ = np.diff(vs_, axis=0) / dt
                    accel_ratios = np.abs(as_) / (accel_limit_ratio * accel_limits)[None, :]
                    accel_max_ratio = np.max(accel_ratios)
                    if accel_max_ratio > 1:
                        as_ /= accel_max_ratio
                    # [N - 1, N_JOINTS]
                    vs_ = vs_[0][None, :] + np.cumsum(as_, axis=0) * dt

                    target_diffs_speed_limited = vs_ * dt
                    # 速度がしきい値より小さければ静止させ無駄なドリフトを避ける
                    # NOTE: スレーブモードを落とさないためには前の速度が十分小さいとき (しきい値は不明) 
                    # にしか静止させてはいけない
                    for i in range(N - 1):
                        if np.all(target_diffs_speed_limited[i] / dt < stopped_velocity_eps):
                            target_diffs_speed_limited[i] = np.zeros_like(
                                target_diffs_speed_limited[i])
                            vs_[i] = target_diffs_speed_limited[i] / dt

                    # [N - 1, N_JOINTS]
                    target_steps_speed_limited = target_steps[0][None, :] + np.cumsum(vs_, axis=0) * dt
                    self.last_control_velocity = vs_

                target_step_speed_limited = target_steps_speed_limited[pd_step]

                # Next step
                pd_step += 1
                if pd_step == N - 1:
                    pd_step = 0
            elif filter_kind == "none":
                target_diff = target_delayed - self.last_control
            else:
                raise ValueError

            sw.lap("2nd speed limit")
            max_ratio = None
            accel_max_ratio = None
            if use_second_speed_limit:
                if filter_kind != "feedback_pd_traj":
                    # 速度制限
                    dt = now - self.last
                    v = target_diff / dt
                    ratio = np.abs(v) / (speed_limit_ratio * speed_limits)
                    max_ratio = np.max(ratio)
                    if max_ratio > 1:
                        v /= max_ratio
                    target_diff_speed_limited = v * dt

                    # 加速度制限
                    a = (v - self.last_control_velocity) / dt
                    accel_ratio = np.abs(a) / (accel_limit_ratio * accel_limits)
                    accel_max_ratio = np.max(accel_ratio)
                    if accel_max_ratio > 1:
                        a /= accel_max_ratio
                    v = self.last_control_velocity + a * dt
                    target_diff_speed_limited = v * dt

                    # 速度がしきい値より小さければ静止させ無駄なドリフトを避ける
                    # NOTE: スレーブモードを落とさないためには前の速度が十分小さいとき (しきい値は不明) 
                    # にしか静止させてはいけない
                    if np.all(target_diff_speed_limited / dt < stopped_velocity_eps):
                        target_diff_speed_limited = np.zeros_like(
                            target_diff_speed_limited)
                        v = target_diff_speed_limited / dt

                    self.last_control_velocity = v
            else:
                target_diff_speed_limited = target_diff

            sw.lap("Get control")
            # 平滑化の種類による対応
            if filter_kind == "original":
                control = self.last_control + target_diff_speed_limited
                # 登録するだけ
                self._filter.filter(control)
            elif filter_kind == "target":
                control = last_target_filtered + target_diff_speed_limited
            elif filter_kind == "filter_target_from_target_but_diff_from_control":
                # 前回制御値との差分を実機はモニタするので、これに対して速度制限をかけることが重要
                control = self.last_control + target_diff_speed_limited
            elif filter_kind == "state_and_target_diff":
                control = last_target_filtered + target_diff_speed_limited
            elif filter_kind == "moveit_servo_humble":
                control = state + target_diff_speed_limited
            elif filter_kind == "control_and_target_diff":
                control = last_target_filtered + target_diff_speed_limited
            elif filter_kind == "feedback_pd_traj":
                control = target_step_speed_limited
            elif filter_kind == "none":
                control = self.last_control + target_diff_speed_limited
            else:
                raise ValueError

            sw.lap("Put control to shared memory")
            self.shm.joint_control = control

            sw.lap("Save control - gather data")
            # 分析用データ保存
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
            t_elapsed = time.time() - now
            if t_elapsed > t_intv * 2:
                self.logger.warning(
                    f"Control loop is 2 times as slow as expected before command: "
                    f"{t_elapsed} seconds")
                self.logger.warning(sw.summary())

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
            t_wait = t_intv - t_elapsed
            if t_wait > 0:
                if self.should_wait_control_loop():
                    time.sleep(t_wait)

            sw.lap("Check elapsed after command")
            t_elapsed = time.time() - now
            if t_elapsed > t_intv * 2:
                self.logger.warning(
                    f"Control loop is 2 times as slow as expected after command: "
                    f"{t_elapsed} seconds")
                self.logger.warning(sw.summary())

            self.control = control
            sw.stop()
            if stop:
                if self.is_ready_to_stop():
                    break

            self.last_control = control
            self.last = now
 
        # スレーブモード解除可能な状態になったら即時に解除しないと指令値生成遅延になる
        self.leave_servo_mode()       

         # ツールチェンジなど後の制御可能フラグ
        self.shm.is_controllable = 0

        hand_thread.join()
        if error_event.is_set():
            # TODO: これで例外発生元のスタックトレースが取得できればこれで十分
            raise error_info['exception']
        return True

    def move_joint_servo(
        self,
        control: List[float],
        lock,
        error_info,
        error_event,
        stop_event,
    ) -> bool:
        assert self.robot is not None
        # ロボット固有の処理を含む
        is_success = self.robot.move_joint_servo_by_pos(*control)
        if not is_success:
            msg = "Failed to send servoJ command"
            with lock:
                error_info['kind'] = "robot"
                error_info['msg'] = msg
                error_info['exception'] = ValueError(msg)
            error_event.set()
            stop_event.set()
        return is_success

    def move_joint_servo_by_vel(
        self,
        control: List[float],
        lock,
        error_info,
        error_event,
        stop_event,
    ) -> bool:
        assert self.robot is not None
        # ロボット固有の処理を含む
        is_success = self.robot.move_joint_servo_by_vel(*control)
        if not is_success:
            msg = "Failed to send servoJ command"
            with lock:
                error_info['kind'] = "robot"
                error_info['msg'] = msg
                error_info['exception'] = ValueError(msg)
            error_event.set()
            stop_event.set()
        return is_success

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

    def send_grip(self) -> None:
        # ロボット固有の処理を含む
        # fully close the hand at half speed and minimum applied force (62.5% of max force is the minimum value that can be set)
        if self.qb_hand is not None:
            self.qb_hand.setClosure(100, 50, 62.5)

    def send_release(self) -> None:
        # ロボット固有の処理を含む
        # reopen at full speed and full force
        if self.qb_hand is not None:
            self.qb_hand.setClosure(0, 100, 100)

    def release_hand(self) -> bool:
        self.logger.info("Release hand")
        try:
            if not self.send_release():
                raise ValueError("Failed to release hand")
            return True
        except Exception as e:
            self.logger.error("Error releasing hand")
            self.logger.error(f"{self.format_error(e)}")
            return False

    def enable(self) -> bool:
        self.logger.info("Enabling robot")
        try:
            if not self.robot.enable():
                raise ValueError("Failed to enable robot")
            return True
        except Exception as e:
            self.logger.error("Error enabling robot")
            self.logger.error(f"{self.format_error(e)}")
            if "ur_rtde: Failed to start control script, before timeout of 5 seconds" in str(e):
                self.logger.error("This error may occur occasionally. Try enabling several times before giving up")
            return False

    def disable(self) -> bool:
        self.logger.info("Disabling robot")
        try:
            if not self.robot.disable():
                raise ValueError("Failed to disable robot")
            return True
        except Exception as e:
            self.logger.error("Error disabling robot")
            self.logger.error(f"{self.format_error(e)}")
            return False

    def set_area_enabled(self, enable: bool) -> bool:
        raise NotImplementedError

    def tidy_pose(self) -> bool:
        self.logger.info("Tidy pose")
        try:
            ret = self.robot.move_joint(*self.tidy_joint)
            if not ret:
                raise ValueError("Failed to move to tidy pose")
            return True
        except Exception as e:
            self.logger.error("Error moving to tidy pose")
            self.logger.error(f"{self.format_error(e)}")
            return False

    def move_joint(self, joints: List[float]) -> bool:
        self.logger.info("Move joint")
        try:
            ret = self.robot.move_joint(*joints)
            if not ret:
                raise ValueError("Failed to move to joint pose")
            return True
        except Exception as e:
            self.logger.error("Error moving to joint pose")
            self.logger.error(f"{self.format_error(e)}")
            return False

    def clear_error(self) -> bool:
        raise NotImplementedError

    def enter_servo_mode(self) -> bool:
        # self.shm.maybe_slave_modeは0のとき必ず通常モード。
        # self.shm.maybe_slave_modeは1のとき基本的にスレーブモードだが、
        # 変化前後の短い時間は通常モードの可能性がある。
        # 順番固定
        with self.slave_mode_lock:
            self.shm.maybe_slave_mode = 1
        return True

    def leave_servo_mode(self) -> bool:
        # self.shm.maybe_slave_modeは0のとき必ず通常モード。
        # self.shm.maybe_slave_modeは1のとき基本的にスレーブモードだが、
        # 変化前後の短い時間は通常モードの可能性がある。
        # 順番固定
        self.shm.maybe_slave_mode = 0
        return True

    def should_recover_automatic_on_timeout_error(self, e_leave) -> bool:
        # ロボット固有の処理を含む
        # NOTE: タイムアウトエラーが発生する場合は実装する
        return False

    def recover_automatic_on_timeout_error(self) -> bool:
        # ロボット固有の処理を含む
        # NOTE: タイムアウトエラーが発生する場合は実装する
        return False

    def recover_automatic_on_recoverable_error(self) -> bool:
        try:
            self.robot.recover_from_recoverable_robot_state()
            ret = self.enable()
            if ret:
                # 自動復帰可能エラー 
                self.logger.info("Automatic recover succeeded")
                return True
            else:
                # 自動復帰不可能エラー
                self.logger.error(
                    "Error is not automatically recoverable")
                return False
        except Exception as e_recover:
            self.logger.error("Error during automatic recover")
            self.logger.error(f"{self.format_error(e_recover)}")
            return False

    def control_loop_w_recover_automatic(self) -> bool:
        """自動復帰を含むリアルタイム制御ループ"""
        self.logger.info("Start Control Loop with Automatic Recover")
        # 自動復帰ループ
        while True:
            try:
                # 制御ループ
                # 停止するのは、ユーザーが要求した場合か、自然に内部エラーが発生した場合
                self.control_loop()
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
                if str(e) == "Target and state are too different":
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
                    # ツールチェンジに成功した場合は、ループを継続し
                    # 失敗した場合は、ループを抜ける
                    try:
                        self.tool_change(next_tool_id)
                        # NOTE: より良い方法がないか
                        # VRアニメーションがロボットの動きに追従し終わるのを待つ
                        time.sleep(3)
                        self.shm.tool_change_result = 1
                        self.shm.tool_change = 0
                        # VRのIKで解いた関節角度にロボットの関節角度を合わせるのを待つ
                        time.sleep(3)
                        self.logger.info("Tool change succeeded")
                    except Exception as e:
                        self.logger.error("Error during tool change")
                        self.logger.error(f"{self.format_error(e)}")
                        self.shm.tool_change_result = 2
                        self.shm.tool_change = 0
                        break
                # 棚の上の箱を置くことが要求された場合
                elif put_down_box != 0:
                    self.logger.info("User required put down box")
                    # 成功しても失敗してもループを継続する (ツールを変えることによる
                    # 予測できないエラーは起こらないため)
                    self.demo_put_down_box()
                elif line_cut != 0:
                    self.logger.info("User required line cut")
                    # 成功しても失敗してもループを継続する (ツールを変えることによる
                    # 予測できないエラーは起こらないため)
                    self.line_cut()
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
        self, tool_infos: List[Dict[str, Any]], tool_id: int) -> Dict[str, Any]:
        return [tool_info for tool_info in tool_infos
                if tool_info["id"] == tool_id][0]

    def tool_change(self, next_tool_id: int) -> None:
        raise NotImplementedError

    def tool_change_not_in_rt(self, tool_id: int) -> bool:
        self.logger.info("Tool change not in real-time")
        raise NotImplementedError

    def jog_joint(self, joint: int, direction: float) -> bool:
        try:
            if self.shm.is_joint_state_received != 1:
                raise ValueError("Joint jog requires joint state to be monitored but currently not")
            # joint state
            joints = self.shm.joint_state.copy()
            joints = np.asarray(joints)
            joints[joint] += direction
            joints = joints.tolist()
            is_success = self.robot.move_joint(*joints)
            if not is_success:
                raise ValueError("move_joint failed")
            return True
        except Exception as e:
            self.logger.error("Error during joint jog")
            self.logger.error(f"{self.format_error(e)}")
            return False

    def jog_tcp(self, axis: int, direction: float) -> bool:
        try:
            if self.shm.is_pose_state_received != 1:
                raise ValueError("TCP jog requires TCP state to be monitored but currently not")
            # TCP state
            poses = self.shm.pose_state.copy()
            poses = np.asarray(poses)
            poses[axis] += direction
            poses = poses.tolist()
            is_success = self.robot.move_pose(*poses)
            if not is_success:
                raise ValueError("move_pose failed")
            return True
        except Exception as e:
            self.logger.error("Error during TCP jog")
            self.logger.error(f"{self.format_error(e)}")
            return False

    def demo_put_down_box(self) -> bool:
        self.logger.info("Demo put down box")
        raise NotImplementedError

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

    def line_cut(self) -> bool:
        self.logger.info("Line cut")
        raise NotImplementedError

    def del_robot(self) -> None:
        self.robot.disable()
        self.robot.stop()

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


class Doosan_CON_Archiver:
    def monitor_start(self, f: TextIO | None = None):
        while True:
            # ログファイル変更時
            if self.shm.change_log_file_control_archiver == 1:
                return True
            try:
                datum = self.control_to_archiver_queue.get(
                    block=True, timeout=T_INTV)
            except queue.Empty:
                datum = None
            if ((f is not None) and 
                (datum is not None)):
                s = ""
                for d in datum:
                    s = s + json.dumps(d, ensure_ascii=False) + "\n"
                f.write(s)
            # プロセス終了時
            if self.shm.exit_program == 1:
                return False

    def setup_logger(self, log_queue):
        self.logger = logging.getLogger("CTRL-ARCV")
        if log_queue is not None:
            self.handler = logging.handlers.QueueHandler(log_queue)
        else:
            self.handler = logging.StreamHandler()
        self.logger.addHandler(self.handler)
        self.logger.setLevel(logging.INFO)

    def get_logging_dir_and_change_log_file(self) -> None:
        command = self.control_arcv_pipe.recv()
        logging_dir = command["params"]["logging_dir"]
        self.logger.info("Change log file")
        self.change_log_file(logging_dir)

    def change_log_file(self, logging_dir: str) -> None:
        self.logging_dir = logging_dir
        self.shm.change_log_file_control_archiver = 0

    def run_proc(self, control_arcv_pipe, log_queue, logging_dir, control_to_archiver_queue):
        self.setup_logger(log_queue)
        self.logger.info("Process started")
        self.shm = NamedSharedMemory(create=False)
        self.control_arcv_pipe = control_arcv_pipe
        self.logging_dir = logging_dir
        self.control_to_archiver_queue = control_to_archiver_queue

        while True:
            try:
                # 基本はmonitor_start内のループにいるが、
                # ログファイル変更またはプロセス終了時に
                # monitor_startから抜ける
                if save_control:
                    with open(
                        os.path.join(self.logging_dir, "control.jsonl"), "a"
                    ) as f:
                        will_change_log_file = self.monitor_start(f)
                else:
                    will_change_log_file = self.monitor_start()
                # ログファイル変更時は大きいループを継続
                if will_change_log_file:
                    self.get_logging_dir_and_change_log_file()
            except Exception as e:
                self.logger.error("Error in control archiver")
                self.logger.error(e)
            # プロセス終了時は大きいループを抜ける
            if self.shm.exit_program == 1:
                self.shm.release()
                self.control_to_archiver_queue.close()
                time.sleep(1)
                self.logger.info("Process stopped")
                self.handler.close()
                break
