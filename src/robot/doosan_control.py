# Doosanを制御する
import logging
import traceback
from typing import Any, Dict, List

import numpy as np

from ..common.control import ControlBase
from ..common.control_config import ControlConfig
from ..common.utils import deg2rad_list
from .config import (
    HAND_IP,
    ROBOT_IP,
    abs_joint_soft_limit,
    control_interface,
    delay_for_interpolation,
    eff_accel_limits,
    eff_speed_limits,
    filter_kind,
    move_robot,
    n_windows,
    N_JOINTS,
    stopped_velocity_eps,
    T_INTV,
    target_state_abs_joint_diff_limit,
    use_first_speed_limit,
    use_interp,
    use_normalize_target_to_nearest,
    use_second_speed_limit,
)
from .doosan_robot import ROBOT_STATE, DoosanRobot
from .qbsofthand_industry_api_pybind import qbSoftHandIndustryAPI
from .tools import tool_classes, tool_infos

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


class Doosan_CON(ControlBase):
    def __init__(self):
        config = ControlConfig(
            n_joints=N_JOINTS,
            t_intv=T_INTV,
            move_robot=move_robot,
            filter_kind=filter_kind,
            n_windows=n_windows,
            use_interp=use_interp,
            delay_for_interpolation=delay_for_interpolation,
            use_first_speed_limit=use_first_speed_limit,
            use_second_speed_limit=use_second_speed_limit,
            eff_speed_limits=eff_speed_limits,
            eff_accel_limits=eff_accel_limits,
            abs_joint_soft_limit=abs_joint_soft_limit,
            target_state_abs_joint_diff_limit=target_state_abs_joint_diff_limit,
            stopped_velocity_eps=stopped_velocity_eps,
            use_normalize_target_to_nearest=use_normalize_target_to_nearest,
            control_interface=control_interface,
        )
        super().__init__(config)
        self.robot_ip = ROBOT_IP
        self.hand_ip = HAND_IP
        self.default_joint = default_joints["vr5"]
        self.tidy_joint = default_joints["tidy"]
        self.robot: DoosanRobot | None = None
        self.qb_hand: qbSoftHandIndustryAPI | None = None
        self.all_robot_state = {}

    def init_robot(self) -> None:
        # TODO: 要改善
        # ロボット固有の処理を含む
        try:
            use_robot_log_loop = True
            use_monitor_loop = True
            if self.robot is None:
                self.robot = DoosanRobot(self.robot_ip, "queue", self.config.t_intv)
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

    def robot_log_loop_step(self) -> None:
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
        # TODO
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
        self.qb_hand = qbSoftHandIndustryAPI(self.hand_ip, max_timeout)
        if not self.qb_hand.isInitialized():
            raise ValueError("Failed to initialize qbSoftHandIndustryAPI")

    def format_error(self, e: Exception) -> str:
        # ロボット固有の処理を含む
        s = "\n"
        s = s + "Error trace: " + traceback.format_exc() + "\n"
        return s

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
            self.send_release()
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
        # TODO: boolかtryか
        # self.shm.maybe_slave_modeは0のとき必ず通常モード。
        # self.shm.maybe_slave_modeは1のとき基本的にスレーブモードだが、
        # 変化前後の短い時間は通常モードの可能性がある。
        # 順番固定
        with self.slave_mode_lock:
            self.shm.maybe_slave_mode = 1
        return True

    def leave_servo_mode(self) -> bool:
        # TODO: boolかtryか
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

    def _tool_change_impl(self, next_tool_id: int) -> None:
        # TODO
        # ロボット固有の処理を含む
        self.logger.info("_tool_change_impl is not implemented")

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

    def line_cut(self) -> bool:
        self.logger.info("Line cut")
        raise NotImplementedError

    def del_robot(self) -> None:
        self.robot.disable()
        self.robot.stop()
