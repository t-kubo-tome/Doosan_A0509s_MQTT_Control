import multiprocessing
from multiprocessing import Process

from common.control_archiver import CON_Archiver
from common.ipc import TopicMemory
from common.monitor_gui import run_joint_monitor_gui
from robot import CON, MON, MQTT_Recv
from robot.config import robot_name, topic_types
from robot.shared_memory import NamedSharedMemory


class ProcessManager:
    """複数プロセスを管理するクラス."""
    def __init__(self, use_command_queue: bool = False):
        # mp.set_start_method('spawn')
        # 共有メモリ
        self.shm = NamedSharedMemory(create=True)
        self.manager = multiprocessing.Manager()
        self.topic_memory = TopicMemory(self.manager, topic_types=topic_types)
        self.slave_mode_lock = multiprocessing.Lock()
        self.main_to_control_pipe, self.control_pipe = multiprocessing.Pipe()
        self.main_to_monitor_pipe, self.monitor_pipe = multiprocessing.Pipe()
        self.log_queue = multiprocessing.Queue()
        self.command_queue = multiprocessing.Queue() if use_command_queue else None
        self.control_to_archiver_queue = multiprocessing.Queue()
        self.main_to_control_archiver_pipe, self.control_archiver_pipe = \
            multiprocessing.Pipe()
        self.monitor_queue = multiprocessing.Queue()
        # プロセス
        self.recvP = None
        self.monP = None
        self.ctrlP = None
        self.monitor_guiP = None
        self.ctrl_archiverP = None

    @property
    def state_recv_mqtt(self) -> bool:
        return self.recvP is not None and self.recvP.is_alive()
        
    @property
    def state_control(self) -> bool:
        return self.ctrlP is not None and self.ctrlP.is_alive()
    
    @property
    def state_control_archiver(self) -> bool:
        return self.ctrl_archiverP is not None and self.ctrl_archiverP.is_alive()

    @property
    def state_monitor(self) -> bool:
        return self.monP is not None and self.monP.is_alive()

    @property
    def state_monitor_gui(self) -> bool:
        return self.monitor_guiP is not None and self.monitor_guiP.is_alive()

    def startRecvMQTT(self):
        self.recv = MQTT_Recv()
        self.recvP = Process(
            target=self.recv.run_proc,
            args=(self.topic_memory,
                  self.log_queue,
                  self.command_queue),
            name="MQTT-recv")
        self.recvP.start()

    def startMonitor(self, logging_dir: str | None = None, disable_mqtt: bool = False):
        self.mon = MON()
        self.monP = Process(
            target=self.mon.run_proc,
            args=(self.topic_memory,
                  self.slave_mode_lock,
                  self.log_queue,
                  self.monitor_pipe,
                  self.monitor_queue,
                  logging_dir,
                  disable_mqtt),
            name=f"{robot_name}-monitor")
        self.monP.start()

    def startControl(self, logging_dir: str | None = None):
        self.ctrl = CON()
        self.ctrlP = Process(
            target=self.ctrl.run_proc,
            args=(self.control_pipe,
                  self.slave_mode_lock,
                  self.log_queue,
                  self.control_to_archiver_queue,
                  self.monitor_queue,
                ),
            name=f"{robot_name}-control")
        self.ctrlP.start()

        self.ctrl_archiver = CON_Archiver()
        self.ctrl_archiverP = Process(
            target=self.ctrl_archiver.run_proc,
            args=(self.control_archiver_pipe,
                  self.log_queue,
                  logging_dir,
                  self.control_to_archiver_queue,
                  ),
            name=f"{robot_name}-control-archiver")
        self.ctrl_archiverP.start()

    def startMonitorGUI(self):
        self.monitor_guiP = Process(
            target=run_joint_monitor_gui,
            name=f"{robot_name}-monitor-gui",
        )
        self.monitor_guiP.start()

    def stop_all_processes(self):
        self.shm.exit_program = 1
        self.shm.stop_realtime_control = 1
        if self.recvP is not None:
            self.recvP.join()
        if self.monP is not None:
            self.monP.join()
        if self.ctrlP is not None:
            self.ctrlP.join()
        if self.ctrl_archiverP is not None:
            self.ctrl_archiverP.join()
        if self.monitor_guiP is not None:
            self.monitor_guiP.join()
        self.shm.release()
        self.manager.shutdown()
        self.main_to_control_pipe.close()
        self.control_pipe.close()
        self.main_to_monitor_pipe.close()
        self.monitor_pipe.close()
        self.control_to_archiver_queue.close()

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
