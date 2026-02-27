import numpy as np

from ..common.shared_memory import NamedSharedMemoryBase


class NamedSharedMemory(NamedSharedMemoryBase):
    name = "doosan"
    size = 64
    # self._arの要素の説明と
    # [0:6]: 関節の状態値。 -> joint_state
    # [6:12]: 関節の目標値。 -> joint_target
    # [12]: ハンドの状態値。 -> hand_state
    # [13]: ハンドの目標値。 -> hand_target
    # [14]: 多分スレーブモードか。0: 必ず通常モード。1: 多分スレーブモード。 -> maybe_slave_mode
    # [15]: MQTT制御中か。0: MQTT制御中でない。1: MQTT制御中。 -> is_mqtt_control
    # [16]: リアルタイム制御を停止する。MQTT制御停止命令ではない。0: 停止しない。1: 停止する。-> stop_realtime_control
    # [17]: ツールチェンジを開始する。0: 開始しない。0以外: 開始する。次のツール番号が入る。 -> tool_change
    # [18]: ツールチェンジの完了状態。0: 未定義。1: 成功。2: 失敗 -> tool_change_result
    # [19]: 共有メモリの関節の状態値を更新したか。0: 未受信。1: 受信。 -> is_joint_state_received
    # [20]: 共有メモリの関節の目標値を更新したか。0: 未受信。1: 受信。 -> is_joint_target_received
    # [21]: 棚の上の箱を作業台に置くデモを開始する。0: 開始しない。1: 開始する。 -> demo_put_down_box
    # [22]: 棚の上の箱を作業台に置くデモの完了状態。0: 未定義。1: 成功。2: 失敗 -> demo_put_down_box_result
    # [23]: 現在のツール番号。 -> current_tool_id
    # [24:30]: 関節の制御値。 -> joint_control
    # [31]: セーフティエリア機能が有効か。0: 無効。1: 有効 -> is_area_enabled
    # [32]: プログラムを終了する。0: 終了しない。1: 終了する。 -> exit_program
    # [34]: ログ出力先を変更する。(monitor用) -> change_log_file_monitor
    # [35]: ログ出力先を変更する。(control-archiver用) -> change_log_file_control_archiver
    # [36]: 非常停止か。0: 非常停止でない。1: 非常停止 -> is_emergency_stopped
    # [37]: スレーブモードの状態値。0: 通常モード。1: スレーブモード -> slave_mode
    # [38]: カッター移動を開始する。0:開始しない。1: 開始 -> line_cut
    # [39]: カッター移動の完了状態。0: 未定義。1: 成功。2: 失敗 -> line_cut_result
    # [40]: ハンドの把持力。 -> hand_force
    # [41]: ツールチェンジなど後の制御可能フラグ。0: 制御不可。1: 制御可能 -> is_controllable
    # [42:48]: TCP姿勢 -> pose_state
    # [48]: 共有メモリのTCP姿勢を受信したか。0: 未受信。1: 受信済み -> is_pose_state_received

    @property
    def joint_state(self) -> np.ndarray:
        return self._ar[0:6]

    @joint_state.setter
    def joint_state(self, value: np.ndarray) -> None:
        self._ar[0:6] = value
