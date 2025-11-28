import numpy as np


def deg2rad(deg):
    return deg * np.pi / 180.0

def deg2rad_list(deg_list: list[float]) -> list[float]:
    return [deg2rad(deg) for deg in deg_list]

def rad2deg(rad):
    return rad * 180.0 / np.pi

def rad2deg_list(rad_list: list[float]) -> list[float]:
    return [rad2deg(rad) for rad in rad_list]


def parse_safety_status_bits(safety_status):
    # 複合的なモードになっている
    # ROBOT_MODEがPOWER_OFFでもRUNNINGでもIS_NORMAL_MODEはTrue
    IS_NORMAL_MODE = (safety_status & (1 << 0)) != 0
    IS_REDUCED_MODE = (safety_status & (1 << 1)) != 0
    IS_PROTECTIVE_STOPPED = (safety_status & (1 << 2)) != 0
    IS_RECOVERY_MODE = (safety_status & (1 << 3)) != 0
    IS_SAFEGUARD_STOPPED = (safety_status & (1 << 4)) != 0
    IS_SYSTEM_EMERGENCY_STOPPED = (safety_status & (1 << 5)) != 0
    IS_ROBOT_EMERGENCY_STOPPED = (safety_status & (1 << 6)) != 0
    IS_EMERGENCY_STOPPED = (safety_status & (1 << 7)) != 0
    IS_VIOLATION = (safety_status & (1 << 8)) != 0
    IS_FAULT = (safety_status & (1 << 9)) != 0
    IS_STOPPED_DUE_TO_SAFETY = (safety_status & (1 << 10)) != 0
    return {
        "IS_NORMAL_MODE": IS_NORMAL_MODE,
        "IS_REDUCED_MODE": IS_REDUCED_MODE,
        "IS_PROTECTIVE_STOPPED": IS_PROTECTIVE_STOPPED,
        "IS_RECOVERY_MODE": IS_RECOVERY_MODE,
        "IS_SAFEGUARD_STOPPED": IS_SAFEGUARD_STOPPED,
        "IS_SYSTEM_EMERGENCY_STOPPED": IS_SYSTEM_EMERGENCY_STOPPED,
        "IS_ROBOT_EMERGENCY_STOPPED": IS_ROBOT_EMERGENCY_STOPPED,
        "IS_EMERGENCY_STOPPED": IS_EMERGENCY_STOPPED,
        "IS_VIOLATION": IS_VIOLATION,
        "IS_FAULT": IS_FAULT,
        "IS_STOPPED_DUE_TO_SAFETY": IS_STOPPED_DUE_TO_SAFETY,
    }


robot_modes = {
    -1: "ROBOT_MODE_NO_CONTROLLER",
    0: "ROBOT_MODE_DISCONNECTED",
    1: "ROBOT_MODE_CONFIRM_SAFETY",
    2: "ROBOT_MODE_BOOTING",
    3: "ROBOT_MODE_POWER_OFF",
    4: "ROBOT_MODE_POWER_ON",
    5: "ROBOT_MODE_IDLE",
    6: "ROBOT_MODE_BACKDRIVE",
    7: "ROBOT_MODE_RUNNING",
    8: "ROBOT_MODE_UPDATING_FIRMWARE",
}


def parse_robot_mode(robot_mode):
    # 排他的なモードになっている
    # ROBOT_MODEのPOWER_OFF > BOOTING > IDLE > RUNNING
    # TPのRobot Statusは
    # Power OFF > Booting > Robot Idle > Brakes Release > Robot operational
    # Power ON > Booting Complete > Robot Active > Brakes Released > Robot in Normal Mode
    return robot_modes[robot_mode]


def parse_robot_mode_as_dict(robot_mode):
    return {mode_name: mode_value == robot_mode for mode_value, mode_name in robot_modes.items()}


def parse_robot_status(robot_status):
    # robot_statusはrtde_c/rtde_r/rtde_d.getRobotStatus()の戻り値
    # 各クライアントによって同期が取れていない可能性があるのでrtde_c推奨
    # 複合的なモードになっている
    IS_POWER_ON = (robot_status & (1 << 0)) != 0
    IS_PROGRAM_RUNNING = (robot_status & (1 << 1)) != 0
    IS_TEACH_BUTTON_PRESSED = (robot_status & (1 << 2)) != 0
    IS_POWER_BUTTON_PRESSED = (robot_status & (1 << 3)) != 0
    return {
        "IS_POWER_ON": IS_POWER_ON,
        "IS_PROGRAM_RUNNING": IS_PROGRAM_RUNNING,
        "IS_TEACH_BUTTON_PRESSED": IS_TEACH_BUTTON_PRESSED,
        "IS_POWER_BUTTON_PRESSED": IS_POWER_BUTTON_PRESSED,
    }


def rtde_d_batch_monitor(rtde_d):
    # 50ms程度かかる
    return {
        # type: bool
        "isConnected": rtde_d.isConnected(),
        # type: bool
        "isInRemoteControl": rtde_d.isInRemoteControl(),
        # type: str, example: 'Safetystatus: NORMAL'
        "safetystatus": rtde_d.safetystatus(),
        # type: str, example: 'RUNNING <unnamed>'
        "programState": rtde_d.programState(),
        # type: str, example: 'Robotmode: RUNNING'
        "robotmode": rtde_d.robotmode(),
    }


def rtde_r_batch_monitor(rtde_r):
    # 0.5ms程度かかる
    getSafetyStatusBits = rtde_r.getSafetyStatusBits()
    getRobotMode = rtde_r.getRobotMode()
    getRobotStatus = rtde_r.getRobotStatus()
    getActualQ = rtde_r.getActualQ()
    return {
        "isConnected": rtde_r.isConnected(),
        "getSafetyStatusBits": getSafetyStatusBits,
        "getSafetyStatusBits_parsed": parse_safety_status_bits(getSafetyStatusBits),
        "getRobotMode": getRobotMode,
        "getRobotMode_parsed": parse_robot_mode(getRobotMode),
        "getRobotStatus": getRobotStatus,
        "getRobotStatus_parsed": parse_robot_status(getRobotStatus),
        "isProtectiveStopped": rtde_r.isProtectiveStopped(),
        "isEmergencyStopped": rtde_r.isEmergencyStopped(),
        "getActualTCPForce": rtde_r.getActualTCPForce(),
        "getFtRawWrench": rtde_r.getFtRawWrench(),
        "getActualQ": getActualQ,
        "getActualQ_degrees": rad2deg_list(getActualQ),
        "getActualTCPPose": rtde_r.getActualTCPPose(),
    }


def rtde_r_batch_monitor_status_only(rtde_r) -> dict[str, bool]:
    # getSafetyStatusBits (複数)
    "IS_NORMAL_MODE"
    "IS_REDUCED_MODE"
    "IS_PROTECTIVE_STOPPED"  # D1
    "IS_RECOVERY_MODE"
    "IS_SAFEGUARD_STOPPED"
    "IS_SYSTEM_EMERGENCY_STOPPED"
    "IS_ROBOT_EMERGENCY_STOPPED"
    "IS_EMERGENCY_STOPPED"  # D2
    "IS_VIOLATION"
    "IS_FAULT"
    "IS_STOPPED_DUE_TO_SAFETY"
    # getRobotMode (排他的)
    "ROBOT_MODE_NO_CONTROLLER"
    "ROBOT_MODE_DISCONNECTED"
    "ROBOT_MODE_CONFIRM_SAFETY"
    "ROBOT_MODE_BOOTING"
    "ROBOT_MODE_POWER_OFF"
    "ROBOT_MODE_POWER_ON"  # D3
    "ROBOT_MODE_IDLE"
    "ROBOT_MODE_BACKDRIVE"
    "ROBOT_MODE_RUNNING"  # D4?
    "ROBOT_MODE_UPDATING_FIRMWARE"
    # getRobotStatus (複数)
    "IS_POWER_ON"  # D3
    "IS_PROGRAM_RUNNING"  # D4?
    "IS_TEACH_BUTTON_PRESSED"  # 不要?
    "IS_POWER_BUTTON_PRESSED"  # 不要?
    "isProtectiveStopped"  # D1
    "isEmergencyStopped"  # D2

    # 0.5ms程度かかる
    getSafetyStatusBits = rtde_r.getSafetyStatusBits()
    getRobotMode = rtde_r.getRobotMode()
    getRobotStatus = rtde_r.getRobotStatus()
    
    ret = {}
    ret.update(parse_safety_status_bits(getSafetyStatusBits))
    ret.update(parse_robot_mode_as_dict(getRobotMode))
    ret.update(parse_robot_status(getRobotStatus))
    ret["isProtectiveStopped"] = rtde_r.isProtectiveStopped()
    ret["isEmergencyStopped"] = rtde_r.isEmergencyStopped()
    return ret


def rtde_c_batch_monitor(rtde_c):
    return {
        "isConnected": rtde_c.isConnected(),
    }


def pose_m_rad_to_mm_deg(pose: list[float]) -> list[float]:
    # RTDEのTCP姿勢はメートル・ラジアン単位なので、ミリ・度単位に変換する
    converted = [
        pose[0] * 1000.0,
        pose[1] * 1000.0,
        pose[2] * 1000.0,
        rad2deg(pose[3]),
        rad2deg(pose[4]),
        rad2deg(pose[5]),
    ]
    return converted


def pose_mm_deg_to_m_rad(pose: list[float]) -> list[float]:
    # ミリ・度単位のTCP姿勢をRTDEのメートル・ラジアン単位に変換する
    converted = [
        pose[0] / 1000.0,
        pose[1] / 1000.0,
        pose[2] / 1000.0,
        deg2rad(pose[3]),
        deg2rad(pose[4]),
        deg2rad(pose[5]),
    ]
    return converted
