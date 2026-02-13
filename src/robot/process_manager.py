# 複数プロセスを管理する

import multiprocessing
import multiprocessing.shared_memory
from multiprocessing import Process

import numpy as np

from .config import SHM_NAME, SHM_SIZE, ROBOT_NAME
from .doosan_control import Doosan_CON, Doosan_CON_Archiver
from .doosan_monitor import Doosan_MON
from ..common.monitor_gui import run_joint_monitor_gui
from .mqtt_recv import MQTT_Recv


class ProcessManager:
    def __init__(self, use_command_queue: bool = False):
        # mp.set_start_method('spawn')
        sz = SHM_SIZE * np.dtype('float32').itemsize
        try:
            self.sm = multiprocessing.shared_memory.SharedMemory(create=True,size = sz, name=SHM_NAME)
        except FileExistsError:
            self.sm = multiprocessing.shared_memory.SharedMemory(size = sz, name=SHM_NAME)
        # self.arの要素の説明
        # [0:6]: 関節の状態値
        # [6:12]: 関節の目標値
        # [12]: ハンドの状態値
        # [13]: ハンドの目標値
        # [14]: 0: 必ず通常モード。1: 基本的にスレーブモード（通常モードになっている場合もある）
        # [15]: 0: mqtt_control実行中でない。1: mqtt_control実行中
        # [16]: 1: リアルタイム制御停止命令（mqtt_control停止命令ではないことに注意）
        # [17]: ツールチェンジの実行フラグ。0: 終了。0以外: 開始。次のツール番号
        # [18]: ツールチェンジ完了状態。0: 未定義。1: 成功。2: 失敗
        # [19]: 制御開始後の状態値の受信フラグ
        # [20]: 制御開始後の目標値の受信フラグ
        # [21]: 棚の上の箱を作業台に置くデモの実行フラグ。0: 終了。1: 開始
        # [22]: 棚の上の箱を作業台に置くデモの完了状態。0: 未定義。1: 成功。2: 失敗
        # [23]: 現在のツール番号
        # [24:30]: 関節の制御値
        # [31]: エリア機能の有効/無効状態。0: 無効。1: 有効
        # [32]: プロセス終了フラグ
        # [33]: ログ出力先の変更フラグ(control用)
        # [34]: ログ出力先の変更フラグ(monitor用)
        # [35]: ログ出力先の変更フラグ(contol-archiver用)
        # [36]: 0: 非常停止でない。1: 非常停止
        # [37]: スレーブモードの状態値。0: 通常モード。1: スレーブモード
        # [38]: カッター移動の実行フラグ。0: 終了。1: 開始
        # [39]: カッター移動の完了状態。0: 未定義。1: 成功。2: 失敗
        # [40]: ハンドの把持力。
        # [41]: ツールチェンジなど後の制御可能フラグ。0: 制御不可。1: 制御可能
        # [42:48]: TCP姿勢
        # [48]: TCP姿勢受信フラグ。0: 未受信。1: 受信済み
        self.ar = np.ndarray((SHM_SIZE,), dtype=np.dtype("float32"), buffer=self.sm.buf) # 共有メモリ上の Array
        self.ar[:] = 0
        self.manager = multiprocessing.Manager()
        self.monitor_dict = self.manager.dict()
        self.monitor_lock = self.manager.Lock()
        self.mqtt_control_dict = self.manager.dict()
        self.mqtt_control_lock = self.manager.Lock()
        self.slave_mode_lock = multiprocessing.Lock()
        self.main_to_control_pipe, self.control_pipe = multiprocessing.Pipe()
        self.main_to_monitor_pipe, self.monitor_pipe = multiprocessing.Pipe()
        self.state_recv_mqtt = False
        self.state_monitor = False
        self.state_control = False
        self.state_monitor_gui = False
        self.log_queue = multiprocessing.Queue()
        self.command_queue = multiprocessing.Queue() if use_command_queue else None
        self.recvP = None
        self.monP = None
        self.ctrlP = None
        self.monitor_guiP = None
        self.ctrl_archiverP = None
        self.control_to_archiver_queue = multiprocessing.Queue()
        self.main_to_control_archiver_pipe, self.control_archiver_pipe = \
            multiprocessing.Pipe()
        self.monitor_queue = multiprocessing.Queue()

    def startRecvMQTT(self):
        self.recv = MQTT_Recv()
        self.recvP = Process(
            target=self.recv.run_proc,
            args=(self.mqtt_control_dict,
                  self.mqtt_control_lock,
                  self.log_queue,
                  self.command_queue),
            name="MQTT-recv")
        self.recvP.start()
        self.state_recv_mqtt = True

    def startMonitor(self, logging_dir: str | None = None, disable_mqtt: bool = False):
        self.mon = Doosan_MON()
        self.monP = Process(
            target=self.mon.run_proc,
            args=(self.monitor_dict,
                  self.monitor_lock,
                  self.slave_mode_lock,
                  self.log_queue,
                  self.monitor_pipe,
                  self.monitor_queue,
                  logging_dir,
                  disable_mqtt),
            name=f"{ROBOT_NAME}-monitor")
        self.monP.start()
        self.state_monitor = True

    def startControl(self, logging_dir: str | None = None):
        self.ctrl = Doosan_CON()
        self.ctrlP = Process(
            target=self.ctrl.run_proc,
            args=(self.control_pipe,
                  self.slave_mode_lock,
                  self.log_queue,
                  logging_dir,
                  self.control_to_archiver_queue,
                  self.monitor_queue,
                ),
            name=f"{ROBOT_NAME}-control")
        self.ctrlP.start()

        self.ctrl_archiver = Doosan_CON_Archiver()
        self.ctrl_archiverP = Process(
            target=self.ctrl_archiver.run_proc,
            args=(self.control_archiver_pipe,
                  self.log_queue,
                  logging_dir,
                  self.control_to_archiver_queue,
                  ),
            name=f"{ROBOT_NAME}-control-archiver")
        self.ctrl_archiverP.start()
        self.state_control = True

    def startMonitorGUI(self):
        self.monitor_guiP = Process(
            target=run_joint_monitor_gui,
            name=f"{ROBOT_NAME}-monitor-gui",
        )
        self.monitor_guiP.start()
        self.state_monitor_gui = True

    def stop_all_processes(self):
        self.ar[32] = 1
        self.ar[16] = 1
        if self.recvP is not None:
            self.recvP.join()
            print("MQTT receive process joined.")
        if self.monP is not None:
            self.monP.join()
            print("Monitor process joined.")
        if self.ctrlP is not None:
            self.ctrlP.join()
            print("Control process joined.")
        if self.ctrl_archiverP is not None:
            self.ctrl_archiverP.join()
            print("Control archiver process joined.")
        if self.monitor_guiP is not None:
            self.monitor_guiP.join()
            print("Monitor GUI process joined.")
        print("All subprocesses joined.")
        self.sm.close()
        print("Shared memory closed.")
        self.sm.unlink()
        print("Shared memory unlinked.")
        self.manager.shutdown()
        print("Manager shutdown complete.")
        self.main_to_control_pipe.close()
        print("Control main pipe closed.")
        self.control_pipe.close()
        print("Control pipe closed.")
        self.main_to_monitor_pipe.close()
        print("Monitor main pipe closed.")
        self.monitor_pipe.close()
        print("Monitor pipe closed.")
        self.control_to_archiver_queue.close()
        print("Control to archiver queue closed.")

    def _send_command_to_control(self, command):
        wait = command.get("wait", False)
        self.main_to_control_pipe.send(command)
        if wait:
            return self.main_to_control_pipe.recv()

    def _send_command_to_control_archiver(self, command):
        self.main_to_control_archiver_pipe.send(command)

    def _send_command_to_monitor(self, command):
        self.main_to_monitor_pipe.send(command)

    @property
    def state_mqtt_control(self):
        return self.ar[15] == 1

    def get_current_monitor_log(self):
        with self.monitor_lock:
            monitor_dict = self.monitor_dict.copy()
        return monitor_dict

    def get_current_mqtt_control_log(self):
        with self.mqtt_control_lock:
            mqtt_control_dict = self.mqtt_control_dict.copy()
        return mqtt_control_dict

    # MQTT制御コマンド群
    def enable(self):
        return self._send_command_to_control({"command": "enable", "wait": True})

    def disable(self):
        return self._send_command_to_control({"command": "disable", "wait": True})

    def set_area_enabled(self, enable: bool):
        return self._send_command_to_control({"command": "set_area_enabled", "params": {"enable": enable}, "wait": True})

    def tidy_pose(self):
        return self._send_command_to_control({"command": "tidy_pose", "wait": True})

    def clear_error(self):
        return self._send_command_to_control({"command": "clear_error", "wait": True})

    def release_hand(self):
        return self._send_command_to_control({"command": "release_hand", "wait": True})

    def line_cut(self):
        return self._send_command_to_control({"command": "line_cut", "wait": True})

    def start_mqtt_control(self):
        return self._send_command_to_control({"command": "start_mqtt_control", "wait": True})

    def stop_mqtt_control(self):
        # mqtt_control中のみシグナルを出す
        if self.state_mqtt_control:
            self.ar[16] = 1
            status = True
        else:
            status = False
        return {"command": "stop_mqtt_control", "status": status, "message": "", "result": {}}

    def tool_change(self, tool_id: int):
        self.ar[17] = tool_id
        return self._send_command_to_control({"command": "tool_change", "wait": True})

    def jog_joint(self, joint, direction):
        return self._send_command_to_control({"command": "jog_joint", "params": {"joint": joint, "direction": direction}, "wait": False})

    def jog_tcp(self, axis, direction):
        return self._send_command_to_control({"command": "jog_tcp", "params": {"axis": axis, "direction": direction}, "wait": False})

    def move_joint(self, joints: list[float], wait: bool = False):
        return self._send_command_to_control({"command": "move_joint", "params": {"joints": joints}, "wait": wait})

    def demo_put_down_box(self):
        return self._send_command_to_control({"command": "demo_put_down_box", "wait": True})

    def change_log_file(self, logging_dir: str):
        # モニタプロセス
        self.ar[34] = 1
        self._send_command_to_monitor({"command": "change_log_file", "params": {"logging_dir": logging_dir}})
        # 制御プロセス
        self.ar[33] = 1
        self._send_command_to_control({"command": "change_log_file", "params": {"logging_dir": logging_dir}, "wait": True})
        # 制御プロセスはMQTTControl時は一旦停止させる
        self.stop_mqtt_control()
        # 制御記録用プロセス
        self.ar[35] = 1
        self._send_command_to_control_archiver({"command": "change_log_file", "params": {"logging_dir": logging_dir}})
        return {"command": "change_log_file", "status": True, "message": "", "result": {}}
