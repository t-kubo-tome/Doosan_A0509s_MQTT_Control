"""ロボットの設定。ユーザーによる変更は基本的に不要。"""

import os
from typing import Literal

import numpy as np
from dotenv import load_dotenv

load_dotenv(os.path.join(os.path.dirname(__file__), '.env'))

# --- 環境変数（.envで変更するもの） ---
ROBOT_UUID = os.getenv("ROBOT_UUID", "ur-real")
ROBOT_MODEL_ENV = os.getenv("ROBOT_MODEL", "doosan-remote-vr")
MQTT_SERVER = os.getenv("MQTT_SERVER", "sora2.uclab.jp")
ROBOT_IP = os.getenv("ROBOT_IP", "192.168.6.43")
HAND_IP = os.getenv("HAND_IP", "192.168.5.44")
MQTT_CTRL_TOPIC = os.getenv("MQTT_CTRL_TOPIC", "control")
MQTT_MANAGE_TOPIC = os.getenv("MQTT_MANAGE_TOPIC", "mgr")
MQTT_ROBOT_STATE_TOPIC = os.getenv("MQTT_ROBOT_STATE_TOPIC", "robot")
MQTT_MODE = os.getenv("MQTT_MODE", "metawork")
SAVE = os.getenv("SAVE", "true") == "true"
MOVE = os.getenv("MOVE", "true") == "true"

# --- ハードコード設定（ユーザーによる変更は基本的に不要） ---
SHM_NAME = "doosan"
SHM_SIZE = 64
N_JOINTS = 6
ABS_JOINT_LIMIT = [360, 95, 135, 360, 135, 360]
T_INTV = 0.05
ROBOT_NAME = "Doosan"
ROBOT_VENDOR = "Doosan Robotics"
ROBOT_MODEL = "A0509s"
# API/GUIによる指令がサポートされるコマンド
SUPPORTED_COMMANDS_COMMON = [
    "enable",
    "disable",
    "tidy_pose",
    "release_hand",
    "start_mqtt_control",
    "stop_mqtt_control",
    "change_log_file",
    "jog_joint",
    "jog_tcp",
    # "clear_error",
    # "demo_put_down_box",
    # "line_cut",
    # "tool_change",
    # "set_area_enabled",
]
# APIによる指令がサポートされるコマンド
SUPPORTED_COMMANDS_API_ONLY = [
    "shutdown",
    "get_command_list",
    "get_joint_names",
]
# GUIによる指令がサポートされるコマンド
SUPPORTED_COMMANDS_GUI_ONLY = [
    "connect_robot",
    "connect_mqtt",
]

# 平滑化の方法
# NOTE: 実際のVRコントローラとの結合時に、
# 遅延などを考慮すると改良が必要かもしれない。
# そのときのヒントとして残している
filter_kind: Literal[
    "original",
    "target",
    "state_and_target_diff",
    "moveit_servo_humble",
    "control_and_target_diff",
    "feedback_pd_traj",
    "none",
    "filter_target_from_target_but_diff_from_control"
] = "filter_target_from_target_but_diff_from_control"  # "original"
n_windows = 10

# 外部速度。単位は%
speed_normal = 20
speed_tool_change = 2

stopped_velocity_eps = 1e-4

# 目標値が状態値よりこの制限より大きく乖離した場合はロボットを停止させる
# 設定値は典型的なVRコントローラの動きから決定した
target_state_abs_joint_diff_limit = [30, 30, 40, 40, 40, 60]

use_normalize_target_to_nearest = True

use_interp = True
delay_for_interpolation = 0.1

use_first_speed_limit = True
use_second_speed_limit = True
speed_limits = [180, 180, 180, 360, 360, 360]
speed_limit_ratio = 0.5
accel_limits = [s ** 2 for s in speed_limits]
accel_limit_ratio = 0.5

control_interface: Literal["position", "velocity"] = "velocity"

move_robot = MOVE
save_control = SAVE

tidy_joint = [0.0, 0.0, -90.0, 0.0, -90.0, 0.0]

# ロボット制御コード内で監視する情報
topic_types = ["mgr/register", "dev", "robot", "control"]

# ロボット固有のパラメータ
servo_mode = 0x202

# パラメータの後処理
speed_limits = np.array(speed_limits)
eff_speed_limits = speed_limits * speed_limit_ratio
accel_limits = np.array(accel_limits)
eff_accel_limits = accel_limits * accel_limit_ratio
abs_joint_limit = ABS_JOINT_LIMIT
abs_joint_limit = np.array(abs_joint_limit)
abs_joint_soft_limit = abs_joint_limit - 10
