# Doosanを制御する
import traceback
from typing import Any, Dict, List

from common.control_hardware_interface import ControlHardwareInterface
from robot import config
from robot.doosan_robot_ext import ROBOT_STATE, DoosanRobotExt


class DoosanControlHardware(ControlHardwareInterface):
    # BEGIN: 実装必須

    # BEGIN: 制御処理

    def on_init(self) -> None:
        """追加の初期化処理。"""
        self.robot = DoosanRobotExt(
            config.robot_ip,
            "queue",
            # 制御周期
            config.t_intv,
            logger=self.process.logger,
            # ログ収集周期は制御より優先度を下げる (周期を長くとる)
            log_t_intv=config.t_intv * 2,
        )

    def on_del(self) -> None:
        """追加の終了処理。"""
        self.robot.stop_log_if_exists()

    def format_error(self, e: Exception) -> str:
        """例外をフォーマットする。"""
        s = "Error trace: " + "\n" + traceback.format_exc()
        return s

    def start(self) -> bool:
        """ロボットに接続する。"""
        return self.robot.start()

    def stop(self) -> bool:
        """ロボットの接続を停止する。"""
        return self.robot.stop()

    def enable(self) -> bool:
        """ロボットのモーターの電源をONにする。"""
        return self.robot.enable()

    def disable(self) -> bool:
        """ロボットのモーターの電源をOFFにする。"""
        return self.robot.disable()

    def enter_servo_mode(self) -> None:
        """ロボットをサーボモードに切り替える。"""
        pass

    def leave_servo_mode(self) -> None:
        """ロボットをサーボモードから離れる。"""
        pass

    def move_joint(self, joints: List[float]) -> bool:
        """ロボットを関節空間で移動させる。"""
        return self.robot.move_joint(*joints)

    def move_pose(self, poses: List[float]) -> bool:
        """ロボットをTCP空間で移動させる。"""
        return self.robot.move_pose(*poses)

    def move_joint_servo(self, control: List[float]) -> bool:
        """関節のスレーブモードでの制御値をロボットに送る。"""
        return self.robot.move_joint_servo_by_pos(*control)

    def move_joint_servo_by_vel(self, control: List[float]) -> bool:
        """関節のスレーブモードでの速度制御値をロボットに送る。"""
        return self.robot.move_joint_servo_by_vel(*control)
    
    # END: 制御処理

    # BEGIN: 状態取得

    def get_current_pose_rt(self) -> List[float]:
        """現在のTCPの姿勢を取得する。"""
        return self.robot.get_current_pose_rt()[1:]

    def get_current_joint_rt(self) -> List[float]:
        """現在の関節角度を取得する。"""
        return self.robot.get_current_joint_rt()[1:]

    def get_current_force_rt(self) -> List[float]:
        """現在のTCPの外力を取得する。"""
        return self.robot.get_current_external_tcp_force_rt()[1:]

    def get_all_robot_state_at_once(self) -> None:
        """ロボットの状態値が個別の関数ではなく少数の関数でまとめて取得できる場合に使用。"""
        self.all_robot_state["robot_state"] = self.robot.get_robot_state()

    def get_enabled(self) -> bool:
        """ロボットのモーターの電源がONかどうかを取得する。"""
        robot_state = self.all_robot_state["robot_state"]
        return robot_state in [
            ROBOT_STATE.STATE_STANDBY,
            ROBOT_STATE.STATE_MOVING,
            ROBOT_STATE.STATE_TEACHING,
            ROBOT_STATE.STATE_HOMMING,
        ]

    def get_is_in_servo_mode(self) -> bool:
        """ロボットがサーボモードかどうかを取得する。"""
        # Doosanではスレーブモードの状態はAPIでは不明なので制御値を使用
        return bool(self.shm.maybe_slave_mode)

    def get_is_emergency_stopped(self) -> bool:
        """ロボットが非常停止状態かどうかを取得する。"""
        robot_state = self.all_robot_state["robot_state"]
        return robot_state == ROBOT_STATE.STATE_EMERGENCY_STOP

    def get_errors(self) -> List[Dict[str, Any]]:
        """ロボットのエラーを取得する。"""
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

    # END: 実装必須

    # BEGIN: オーバーライド可能

    def recover_automatic_on_recoverable_error(self) -> bool:
        """回復可能なエラーが発生したときに自動的に回復する処理を行う。"""
        self.robot.recover_from_recoverable_robot_state()
        ret = self.robot.enable()
        return ret

    # END: オーバーライド
