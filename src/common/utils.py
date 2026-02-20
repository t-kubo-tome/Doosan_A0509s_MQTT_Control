import time

import numpy as np


def deg2rad(deg):
    return deg * np.pi / 180.0

def deg2rad_list(deg_list: list[float]) -> list[float]:
    return [deg2rad(deg) for deg in deg_list]

def rad2deg(rad):
    return rad * 180.0 / np.pi

def rad2deg_list(rad_list: list[float]) -> list[float]:
    return [rad2deg(rad) for rad in rad_list]


class StopWatch:
    """ラップとスプリットを記録できるストップウォッチ."""
    def __init__(self):
        self.start_t = None
        self.last_t = None
        self.last_msg = None
        self.laps = []

    def start(self, msg: str = "") -> None:
        t = time.perf_counter()
        self.start_t = t
        self.last_t = t
        self.last_msg = msg
        self.laps = []

    def lap(self, msg: str = "") -> None:
        if ((self.start_t is None) or
            (self.last_t is None) or
            (self.last_msg is None)):
            raise RuntimeError("StopWatch has not been started.")
        t = time.perf_counter()
        self.laps.append({
            "msg": self.last_msg,
            "lap": t - self.last_t,
            "split": t - self.start_t,
        })
        self.last_t = t
        self.last_msg = msg

    def stop(self) -> None:
        self.lap()
        self.start_t = None
        self.last_t = None
        self.last_msg = None
    
    def summary(self) -> str:
        s = "StopWatch summary:\n"
        s += "| No. | Lap (ms) | Split (ms) | Message |\n"
        s += "| - | - | - | - |\n"
        for i, lap in enumerate(self.laps):
            s += f"| {i} | {lap['lap']*1000:.3f} | {lap['split']*1000:.3f} | {lap['msg']} |\n"
        return s
