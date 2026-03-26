from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Any, Dict, List, TYPE_CHECKING

from common.utils import conditional_abstractmethod
from robot import config

if TYPE_CHECKING:
    from common.control_process import ControlProcess


class ControlHardwareInterface(ABC):
    """
    ロボットの制御ループの基底クラス。
    呼び出し元のControlProcessのインスタンス変数の読み書きを、
    self.process.変数名で行うことができる。
    """
    # BEGIN: オーバーライド不可
    
    def __init__(self, process: ControlProcess) -> None:
        self.process = process
        self.on_init()

    # END: オーバーライド不可

    # BEGIN: 実装必須

    # BEGIN: 制御処理

    @abstractmethod
    def on_init(self) -> None:
        """追加の初期化処理。"""
        pass

    @abstractmethod
    def on_del(self) -> None:
        """追加の終了処理。"""
        pass

    @abstractmethod
    def format_error(self, e: Exception) -> str:
        """例外をフォーマットする。"""
        pass

    @abstractmethod
    def start(self) -> bool:
        """ロボットに接続する。"""
        pass

    @abstractmethod
    def stop(self) -> bool:
        """ロボットの接続を停止する。"""
        pass

    @abstractmethod
    def enable(self) -> bool:
        """ロボットのモーターの電源をONにする。"""
        pass

    @abstractmethod
    def disable(self) -> bool:
        """ロボットのモーターの電源をOFFにする。"""
        pass

    @abstractmethod
    def enter_servo_mode(self) -> None:
        """ロボットをサーボモードに切り替える。"""
        pass

    @abstractmethod
    def leave_servo_mode(self) -> None:
        """ロボットをサーボモードから離れる。"""
        pass

    @abstractmethod
    def move_joint(self, joints: List[float]) -> bool:
        """ロボットを関節空間で移動させる。"""
        pass

    @abstractmethod
    def move_pose(self, poses: List[float]) -> bool:
        """ロボットをTCP空間で移動させる。"""
        pass

    @conditional_abstractmethod(config.control_interface == "position")
    def move_joint_servo(
        self,
        control: List[float],
        lock,
        error_info,
        error_event,
        stop_event,
    ) -> bool:
        """関節のスレーブモードでの制御値をロボットに送る。"""
        pass

    @conditional_abstractmethod(config.control_interface == "velocity")
    def move_joint_servo_by_vel(
        self,
        control: List[float],
        lock,
        error_info,
        error_event,
        stop_event,
    ) -> bool:
        """関節のスレーブモードでの速度制御値をロボットに送る。"""
        pass

    # END: 制御処理

    # BEGIN: 状態取得

    @abstractmethod
    def get_current_pose_rt(self) -> List[float]:
        """現在のTCPの姿勢を取得する。"""
        pass

    @abstractmethod
    def get_current_joint_rt(self) -> List[float]:
        """現在の関節角度を取得する。"""
        pass

    @abstractmethod
    def get_current_force_rt(self) -> List[float]:
        """現在のTCPの外力を取得する。"""
        pass

    @abstractmethod
    def get_all_robot_state_at_once(self) -> None:
        """ロボットの状態値が個別の関数ではなく少数の関数でまとめて取得できる場合に使用。"""
        pass

    @abstractmethod
    def get_enabled(self) -> bool:
        """ロボットのモーターの電源がONかどうかを取得する。"""
        pass

    @abstractmethod
    def get_is_in_servo_mode(self) -> bool:
        """ロボットがサーボモードかどうかを取得する。"""
        pass

    @abstractmethod
    def get_is_emergency_stopped(self) -> bool:
        """ロボットが非常停止状態かどうかを取得する。"""
        pass

    @abstractmethod
    def get_errors(self) -> List[Dict[str, Any]]:
        """ロボットのエラーを取得する。"""
        pass

    # END: 状態取得

    # END: 実装必須

    # BEGIN: オーバーライド可能

    def clear_error(self) -> bool:
        """ロボットのエラーをクリアする。"""
        return True

    def set_area_enabled(self, enable: bool) -> bool:
        """ロボットのエリア制限をON/OFFする。"""
        return True

    def _tool_change_impl(self, next_tool_id: int) -> bool:
        """ツールを切り替える。"""
        return False

    def _demo_put_down_box_impl(self) -> bool:
        """箱を動かすデモ。"""
        return False

    def _line_cut_impl(self) -> bool:
        """箱を切るデモ。"""
        return False

    def should_recover_automatic_on_timeout_error(self, e_leave) -> bool:
        """タイムアウトエラーが発生したときに自動的に回復するかどうか。"""
        return False

    def recover_automatic_on_timeout_error(self) -> bool:
        """タイムアウトエラーが発生したときに自動的に回復する処理を行う。"""
        return False

    def recover_automatic_on_recoverable_error(self) -> bool:
        """回復可能なエラーが発生したときに自動的に回復する処理を行う。"""
        return False

    def on_step_start_in_control_loop(self) -> None:
        """制御ループのステップ開始時に呼ばれる。例外の送出は禁止。"""
        pass

    def should_wait_control_loop(self) -> bool:
        """制御ループのステップ開始時に、制御ループを待つべきかどうか。例外の送出は禁止。"""
        return True

    # END: オーバーライド可能