import numpy as np

from ..common.shared_memory import NamedSharedMemoryBase


class NamedSharedMemory(NamedSharedMemoryBase):
    name = "doosan"
    size = 64

    @property
    def joint_state(self) -> np.ndarray:
        """関節の状態値。"""
        return self._ar[0:6]

    @joint_state.setter
    def joint_state(self, value: np.ndarray | list[float]) -> None:
        self._ar[0:6] = value

    @property
    def joint_target(self) -> np.ndarray:
        """関節の目標値。"""
        return self._ar[6:12]

    @joint_target.setter
    def joint_target(self, value: np.ndarray | list[float]) -> None:
        self._ar[6:12] = value

    @property
    def hand_state(self) -> float:
        """ハンドの状態値。"""
        return float(self._ar[12])

    @hand_state.setter
    def hand_state(self, value: float) -> None:
        self._ar[12] = value

    @property
    def hand_target(self) -> float:
        """ハンドの目標値。"""
        return float(self._ar[13])

    @hand_target.setter
    def hand_target(self, value: float) -> None:
        self._ar[13] = value

    @property
    def maybe_slave_mode(self) -> float:
        """多分スレーブモードか。0: 必ず通常モード。1: 多分スレーブモード。"""
        return float(self._ar[14])

    @maybe_slave_mode.setter
    def maybe_slave_mode(self, value: float) -> None:
        self._ar[14] = value

    @property
    def is_mqtt_control(self) -> float:
        """MQTT制御中か。0: MQTT制御中でない。1: MQTT制御中。"""
        return float(self._ar[15])

    @is_mqtt_control.setter
    def is_mqtt_control(self, value: float) -> None:
        self._ar[15] = value

    @property
    def stop_realtime_control(self) -> float:
        """リアルタイム制御を停止する。MQTT制御停止命令ではない。0: 停止しない。1: 停止する。"""
        return float(self._ar[16])

    @stop_realtime_control.setter
    def stop_realtime_control(self, value: float) -> None:
        self._ar[16] = value

    @property
    def tool_change(self) -> float:
        """ツールチェンジを開始する。0: 開始しない。0以外: 開始する。次のツール番号が入る。"""
        return float(self._ar[17])

    @tool_change.setter
    def tool_change(self, value: float) -> None:
        self._ar[17] = value

    @property
    def tool_change_result(self) -> float:
        """ツールチェンジの完了状態。0: 未定義。1: 成功。2: 失敗。"""
        return float(self._ar[18])

    @tool_change_result.setter
    def tool_change_result(self, value: float) -> None:
        self._ar[18] = value

    @property
    def is_joint_state_received(self) -> float:
        """共有メモリの関節の状態値を更新したか。0: 未受信。1: 受信。"""
        return float(self._ar[19])

    @is_joint_state_received.setter
    def is_joint_state_received(self, value: float) -> None:
        self._ar[19] = value

    @property
    def is_joint_target_received(self) -> float:
        """共有メモリの関節の目標値を更新したか。0: 未受信。1: 受信。"""
        return float(self._ar[20])

    @is_joint_target_received.setter
    def is_joint_target_received(self, value: float) -> None:
        self._ar[20] = value

    @property
    def demo_put_down_box(self) -> float:
        """棚の上の箱を作業台に置くデモを開始する。0: 開始しない。1: 開始する。"""
        return float(self._ar[21])

    @demo_put_down_box.setter
    def demo_put_down_box(self, value: float) -> None:
        self._ar[21] = value

    @property
    def demo_put_down_box_result(self) -> float:
        """棚の上の箱を作業台に置くデモの完了状態。0: 未定義。1: 成功。2: 失敗。"""
        return float(self._ar[22])

    @demo_put_down_box_result.setter
    def demo_put_down_box_result(self, value: float) -> None:
        self._ar[22] = value

    @property
    def tool_id(self) -> float:
        """ツール番号。"""
        return float(self._ar[23])

    @tool_id.setter
    def tool_id(self, value: float) -> None:
        self._ar[23] = value

    @property
    def joint_control(self) -> np.ndarray:
        """関節の制御値。"""
        return self._ar[24:30]

    @joint_control.setter
    def joint_control(self, value: np.ndarray | list[float]) -> None:
        self._ar[24:30] = value

    @property
    def is_area_enabled(self) -> float:
        """セーフティエリア機能が有効か。0: 無効。1: 有効。"""
        return float(self._ar[31])

    @is_area_enabled.setter
    def is_area_enabled(self, value: float) -> None:
        self._ar[31] = value

    @property
    def exit_program(self) -> float:
        """プログラムを終了する。0: 終了しない。1: 終了する。"""
        return float(self._ar[32])

    @exit_program.setter
    def exit_program(self, value: float) -> None:
        self._ar[32] = value

    @property
    def change_log_file_monitor(self) -> float:
        """ログ出力先を変更する。(monitor用)"""
        return float(self._ar[34])

    @change_log_file_monitor.setter
    def change_log_file_monitor(self, value: float) -> None:
        self._ar[34] = value

    @property
    def change_log_file_control_archiver(self) -> float:
        """ログ出力先を変更する。(control-archiver用)"""
        return float(self._ar[35])

    @change_log_file_control_archiver.setter
    def change_log_file_control_archiver(self, value: float) -> None:
        self._ar[35] = value

    @property
    def is_emergency_stopped(self) -> float:
        """非常停止か。0: 非常停止でない。1: 非常停止。"""
        return float(self._ar[36])

    @is_emergency_stopped.setter
    def is_emergency_stopped(self, value: float) -> None:
        self._ar[36] = value

    @property
    def slave_mode(self) -> float:
        """スレーブモードの状態値。0: 通常モード。1: スレーブモード。"""
        return float(self._ar[37])

    @slave_mode.setter
    def slave_mode(self, value: float) -> None:
        self._ar[37] = value

    @property
    def line_cut(self) -> float:
        """カッター移動を開始する。0: 開始しない。1: 開始。"""
        return float(self._ar[38])

    @line_cut.setter
    def line_cut(self, value: float) -> None:
        self._ar[38] = value

    @property
    def line_cut_result(self) -> float:
        """カッター移動の完了状態。0: 未定義。1: 成功。2: 失敗。"""
        return float(self._ar[39])

    @line_cut_result.setter
    def line_cut_result(self, value: float) -> None:
        self._ar[39] = value

    @property
    def hand_force(self) -> float:
        """ハンドの把持力。"""
        return float(self._ar[40])

    @hand_force.setter
    def hand_force(self, value: float) -> None:
        self._ar[40] = value

    @property
    def is_controllable(self) -> float:
        """ツールチェンジなどの後でリアルタイム制御が可能になったタイミングを知らせるためのフラグ。0: 制御不可。1: 制御可能。"""
        return float(self._ar[41])

    @is_controllable.setter
    def is_controllable(self, value: float) -> None:
        self._ar[41] = value

    @property
    def pose_state(self) -> np.ndarray:
        """TCP姿勢。"""
        return self._ar[42:48]

    @pose_state.setter
    def pose_state(self, value: np.ndarray | list[float]) -> None:
        self._ar[42:48] = value

    @property
    def is_pose_state_received(self) -> float:
        """共有メモリのTCP姿勢を受信したか。0: 未受信。1: 受信済み。"""
        return float(self._ar[48])

    @is_pose_state_received.setter
    def is_pose_state_received(self, value: float) -> None:
        self._ar[48] = value
