# Doosanを制御する
import os
import traceback
from typing import Any, Dict, List

import numpy as np

from common.control_base import ControlBase
from robot import config
from robot.doosan_robot_ext import ROBOT_STATE, DoosanRobotExt
from robot.tools import tool_classes, tool_infos


class Doosan_CON(ControlBase):

    # BEGIN: 汎用

    def _on_init(self) -> None:
        self.robot: DoosanRobotExt | None = None

    def format_error(self, e: Exception) -> str:
        s = "Error trace: " + "\n" + traceback.format_exc()
        return s

    def del_robot(self) -> None:
        if self.robot is not None:
            self.robot.disable()
            self.robot.stop()
            self.robot.stop_log_if_exists()

    # END: 汎用

    # BEGIN: 受信コマンド

    def connect_robot(self) -> bool:
        try:
            # NOTE: robot_loggerをrun_processで初期化するため、
            # DoosanRobotExtを__init__内で初期化できないため、
            # このような実装にしている
            if self.robot is None:
                self.robot = DoosanRobotExt(
                    config.robot_ip,
                    "queue",
                    config.t_intv,
                    logger=self.robot_logger,
                    # ログは制御より優先度を下げる (周期を長くとる)
                    log_t_intv=config.t_intv * 2,
                )
            if not self.robot.start():
                raise ValueError("Failed to start robot")
            # NOTE: all_robot_stateはDoosanRobotExtの中に組み込めるかも
            self.all_robot_state = {}
            self.init_monitor_loop()
            tool_id = int(os.environ["TOOL_ID"])
            self.find_and_setup_hand(tool_id)
            return True
        except Exception as e:
            self.logger.error("Error in initializing robot: ")
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
            ret = self.robot.move_joint(*config.tidy_joint)
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

    def _tool_change_impl(self, next_tool_id: int) -> None:
        self.logger.info("_tool_change_impl")
        raise NotImplementedError

    def demo_put_down_box(self) -> bool:
        self.logger.info("Demo put down box")
        raise NotImplementedError

    def line_cut(self) -> bool:
        self.logger.info("Line cut")
        raise NotImplementedError

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

    # END: 受信コマンド

    # BEGIN: MQTT制御

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
        # NOTE: タイムアウトエラーが発生する場合は実装する
        return False

    def recover_automatic_on_timeout_error(self) -> bool:
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

    # END: MQTT制御

    # BEGIN: リアルタイム制御

    def move_joint_servo(
        self,
        control: List[float],
        lock,
        error_info,
        error_event,
        stop_event,
    ) -> bool:
        assert self.robot is not None
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

    # END: リアルタイム制御

    # BEGIN: 状態取得

    def get_current_pose_rt(self) -> List[float]:
        return self.robot.get_current_pose_rt()[1:]

    def get_current_joint_rt(self) -> List[float]:
        return self.robot.get_current_joint_rt()[1:]

    def get_current_force_rt(self) -> List[float]:
        return self.robot.get_current_external_tcp_force_rt()[1:]

    def get_all_robot_state_at_once(self) -> None:
        """ロボットの状態値が個別の関数ではなく少数の関数でまとめて取得できる場合に使用"""
        self.all_robot_state["robot_state"] = self.robot.get_robot_state()

    def get_enabled(self) -> bool:
        robot_state = self.all_robot_state["robot_state"]
        return robot_state in [
            ROBOT_STATE.STATE_STANDBY,
            ROBOT_STATE.STATE_MOVING,
            ROBOT_STATE.STATE_TEACHING,
            ROBOT_STATE.STATE_HOMMING,
        ]

    def get_is_in_servo_mode(self) -> bool:
        # Doosanではスレーブモードの状態はAPIでは不明なので制御値を使用
        return bool(self.shm.maybe_slave_mode)

    def get_is_emergency_stopped(self) -> bool:
        robot_state = self.all_robot_state["robot_state"]
        return robot_state == ROBOT_STATE.STATE_EMERGENCY_STOP

    def get_errors(self) -> List[Dict[str, Any]]:
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

    # END: 状態取得

    # BEGIN: ハンド関連

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

    # END: ハンド関連