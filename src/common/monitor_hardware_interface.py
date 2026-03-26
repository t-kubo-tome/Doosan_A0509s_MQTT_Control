from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Any, Dict, List, TYPE_CHECKING

from common.utils import conditional_abstractmethod
from robot import config

if TYPE_CHECKING:
    from common.monitor_process import MonitorProcess


class MonitorHardwareInterface(ABC):
    """
    ロボットのモニタリングループの基底クラス。
    呼び出し元のMonitorProcessのインスタンス変数の読み書きを、
    self.process.変数名で行うことができる。
    """
    # BEGIN: オーバーライド不可
    
    def __init__(self, process: MonitorProcess) -> None:
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

    # BEGIN: 状態取得

    @conditional_abstractmethod(config.real_monitor_process == "monitor")
    def get_current_pose_rt(self) -> List[float]:
        """現在のTCPの姿勢を取得する。"""
        pass

    @conditional_abstractmethod(config.real_monitor_process == "monitor")
    def get_current_joint_rt(self) -> List[float]:
        """現在の関節角度を取得する。"""
        pass

    @conditional_abstractmethod(config.real_monitor_process == "monitor")
    def get_current_force_rt(self) -> List[float]:
        """現在のTCPの外力を取得する。"""
        pass

    @conditional_abstractmethod(config.real_monitor_process == "monitor")
    def get_all_robot_state_at_once(self) -> None:
        """ロボットの状態値が個別の関数ではなく少数の関数でまとめて取得できる場合に使用。"""
        pass

    @conditional_abstractmethod(config.real_monitor_process == "monitor")
    def get_enabled(self) -> bool:
        """ロボットのモーターの電源がONかどうかを取得する。"""
        pass

    @conditional_abstractmethod(config.real_monitor_process == "monitor")
    def get_is_in_servo_mode(self) -> bool:
        """ロボットがサーボモードかどうかを取得する。"""
        pass

    @conditional_abstractmethod(config.real_monitor_process == "monitor")
    def get_is_emergency_stopped(self) -> bool:
        """ロボットが非常停止状態かどうかを取得する。"""
        pass

    @conditional_abstractmethod(config.real_monitor_process == "monitor")
    def get_errors(self) -> List[Dict[str, Any]]:
        """ロボットのエラーを取得する。"""
        pass

    # END: 状態取得

    # END: 実装必須
