# 複数プロセスを管理する

import multiprocessing
from multiprocessing import Process

from ..common.control_save import CON_Archiver
from ..common.monitor_gui import run_joint_monitor_gui
from ..common.shared_memory import TopicMemory
from .config import ROBOT_NAME
from .doosan_control import Doosan_CON
from .doosan_monitor import Doosan_MON
from .doosan_mqtt_recv import Doosan_MQTT_Recv
from .shared_memory import NamedSharedMemory


class ProcessManager:
    def __init__(self, use_command_queue: bool = False):
        # mp.set_start_method('spawn')
        self.shm = NamedSharedMemory(create=True)
        self.manager = multiprocessing.Manager()
        topic_types = ["mgr/register", "dev", "robot", "control"]
        self.topic_memory = TopicMemory(self.manager, topic_types=topic_types)
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
        self.recv = Doosan_MQTT_Recv()
        self.recvP = Process(
            target=self.recv.run_proc,
            args=(self.topic_memory,
                  self.log_queue,
                  self.command_queue),
            name="MQTT-recv")
        self.recvP.start()
        self.state_recv_mqtt = True

    def startMonitor(self, logging_dir: str | None = None, disable_mqtt: bool = False):
        self.mon = Doosan_MON()
        self.monP = Process(
            target=self.mon.run_proc,
            args=(self.topic_memory,
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
                  self.control_to_archiver_queue,
                  self.monitor_queue,
                ),
            name=f"{ROBOT_NAME}-control")
        self.ctrlP.start()

        self.ctrl_archiver = CON_Archiver()
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
        self.shm.exit_program = 1
        self.shm.stop_realtime_control = 1
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
        self.shm.release()
        print("Shared memory released.")
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
        return self.shm.is_mqtt_control == 1

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
        return self._send_command_to_control({"command": "stop_mqtt_control", "wait": True})

    def tool_change(self, tool_id: int):
        return self._send_command_to_control({"command": "tool_change", "params": {"tool_id": tool_id}, "wait": True})

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
        self.shm.change_log_file_monitor = 1
        self._send_command_to_monitor({"command": "change_log_file", "params": {"logging_dir": logging_dir}})
        # 制御記録用プロセス
        self.shm.change_log_file_control_archiver = 1
        self._send_command_to_control_archiver({"command": "change_log_file", "params": {"logging_dir": logging_dir}})
        return {"command": "change_log_file", "status": True, "message": "", "result": {}}
