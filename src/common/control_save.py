import json
import logging
import os
import queue
import time
from typing import TextIO

from ..robot.config import T_INTV, save_control
from .shared_memory import NamedSharedMemory


class CON_Archiver:
    def monitor_start(self, f: TextIO | None = None) -> bool:
        while True:
            # ログファイル変更時
            if self.shm.change_log_file_control_archiver == 1:
                return True
            try:
                datum = self.control_to_archiver_queue.get(
                    block=True, timeout=T_INTV)
            except queue.Empty:
                datum = None
            if ((f is not None) and 
                (datum is not None)):
                s = ""
                for d in datum:
                    s = s + json.dumps(d, ensure_ascii=False) + "\n"
                f.write(s)
            # プロセス終了時
            if self.shm.exit_program == 1:
                return False

    def setup_logger(self, log_queue) -> None:
        self.logger = logging.getLogger("CTRL-ARCV")
        if log_queue is not None:
            self.handler = logging.handlers.QueueHandler(log_queue)
        else:
            self.handler = logging.StreamHandler()
        self.logger.addHandler(self.handler)
        self.logger.setLevel(logging.INFO)

    def get_logging_dir_and_change_log_file(self) -> None:
        command = self.control_arcv_pipe.recv()
        logging_dir = command["params"]["logging_dir"]
        self.logger.info("Change log file")
        self.change_log_file(logging_dir)

    def change_log_file(self, logging_dir: str) -> None:
        self.logging_dir = logging_dir
        self.shm.change_log_file_control_archiver = 0

    def run_proc(
        self,
        control_arcv_pipe,
        log_queue,
        logging_dir,
        control_to_archiver_queue,    
    ) -> None:
        self.setup_logger(log_queue)
        self.logger.info("Process started")
        self.shm = NamedSharedMemory(create=False)
        self.control_arcv_pipe = control_arcv_pipe
        self.logging_dir = logging_dir
        self.control_to_archiver_queue = control_to_archiver_queue

        while True:
            try:
                # 基本はmonitor_start内のループにいるが、
                # ログファイル変更またはプロセス終了時に
                # monitor_startから抜ける
                if save_control:
                    with open(
                        os.path.join(self.logging_dir, "control.jsonl"), "a"
                    ) as f:
                        will_change_log_file = self.monitor_start(f)
                else:
                    will_change_log_file = self.monitor_start()
                # ログファイル変更時は大きいループを継続
                if will_change_log_file:
                    self.get_logging_dir_and_change_log_file()
            except Exception as e:
                self.logger.error("Error in control archiver")
                self.logger.error(e)
            # プロセス終了時は大きいループを抜ける
            if self.shm.exit_program == 1:
                self.shm.release()
                self.control_to_archiver_queue.close()
                time.sleep(1)
                self.logger.info("Process stopped")
                self.handler.close()
                break
