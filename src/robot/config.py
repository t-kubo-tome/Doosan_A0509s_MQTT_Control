"""ロボット固有の設定。開発者が設定する。ユーザーによる設定は基本的に不要。"""
## ハードコード設定。開発者が変更する可能性あり
## ロボットごとのパラメータ
# ロボットのベンダー名
robot_vendor = "Doosan Robotics"
# ロボットモデル
robot_model = "A0509s"
# GUIに表示されるロボット略称
robot_name = "Doosan"
# API/GUIによる指令がサポートされるコマンド
supported_commands_common = [
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
supported_commands_api_only = [
    "shutdown",
    "get_command_list",
    "get_joint_names",
]
# GUIによる指令がサポートされるコマンド
supported_commands_gui_only = [
    "connect_robot",
    "connect_mqtt",
]
# ロボットの関節数
n_joints = 6
# ロボットの整頓時の関節角度 (deg)
tidy_joint = [0.0, 0.0, -90.0, 0.0, -90.0, 0.0]
# リアルタイム制御の制御周期 (s)
t_intv = 0.05
# ロボット制御コード内で扱う関節角度の単位 ("deg" または "rad")
joint_unit_internal = "deg"
# ロボット制御コード外で扱う関節角度の単位 ("deg" または "rad")
joint_unit_external = "rad"
# ロボットからの状態値の取得を実際にはどのプロセスで行うか
# "control": ロボット制御プロセス (のサブスレッド)
# "monitor": ロボットモニタプロセス
real_monitor_process = "control"
# ロボットの関節の角度の範囲 (deg)
abs_joint_limit = [360, 95, 135, 360, 135, 360]
# 目標値が状態値よりこの制限より大きく乖離した場合はロボットを停止させる角度 (deg)
# 設定値は典型的なVRコントローラの動きから決定した
target_state_abs_joint_diff_limit = [30, 30, 40, 40, 40, 60]
# 状態値に最も近い目標値に360度内に規格化する
use_normalize_target_to_nearest = True
# 遅延を許した線形補間を使用するか
use_interp = True
# 線形補間を使用する場合の、目標値の更新からロボットに送るまでの遅延 (s)
delay_for_interpolation = 0.1
# 平滑化の方法。推奨以外は検証用に残しているだけで動作は保証されない
# "filter_target_from_target_but_diff_from_control": 推奨
# "original"
# "target"
# "state_and_target_diff"
# "moveit_servo_humble"
# "control_and_target_diff"
# "feedback_pd_traj"
# "none"
filter_kind = "filter_target_from_target_but_diff_from_control"
# 平滑化のウィンドウサイズ
n_windows = 10
# 平滑化前の速度制限を使用するか
use_first_speed_limit = True
# 平滑化後の速度制限を使用するか
use_second_speed_limit = True
# 速度上限のスペック値 (deg/s)
speed_limits = [180, 180, 180, 360, 360, 360]
# 速度のソフトリミットのスペック値に対する比率
speed_limit_ratio = 0.5
# 加速度上限のスペック値 (deg/s^2)
accel_limits = [s ** 2 for s in speed_limits]
# 加速度のソフトリミットのスペック値に対する比率
accel_limit_ratio = 0.5
# ロボットが停止しているとみなす速度の閾値 (deg/s)
stopped_velocity_eps = 1e-4
# リアルタイム制御でロボットに送る制御値の種類
# "position": 目標関節角度
# "velocity": 目標関節速度
control_interface = "velocity"
# ロボット制御コード内で監視する情報
topic_types = ["mgr/register", "dev", "robot", "control"]
## 特定のロボットだけに有効なパラメータ
## Cobotta Pro
# ツールチェンジホルダー付近の、リアルタイム制御以外での速度 (外部速度) (%)
speed_tool_change = 2
# それ以外の、リアルタイム制御以外での速度 (外部速度) (%)
speed_normal = 20
# ロボットのサーボモード
servo_mode = 0x202
## 以下はパラメータの処理であり、開発者が変更する必要はない
# 環境変数の読み込み
import os
from dotenv import load_dotenv

load_dotenv(os.path.join(os.path.dirname(__file__), '.env'))

robot_uuid = os.getenv("ROBOT_UUID", "ur-real")
robot_model_env = os.getenv("ROBOT_MODEL", "doosan-remote-vr")
mqtt_server = os.getenv("MQTT_SERVER", "sora2.uclab.jp")
robot_ip = os.getenv("ROBOT_IP", "192.168.6.43")
hand_ip = os.getenv("HAND_IP", "192.168.5.44")
mqtt_ctrl_topic = os.getenv("MQTT_CTRL_TOPIC", "control")
mqtt_manage_topic = os.getenv("MQTT_MANAGE_TOPIC", "mgr")
mqtt_robot_state_topic = os.getenv("MQTT_ROBOT_STATE_TOPIC", "robot")
save = os.getenv("SAVE", "true") == "true"
move = os.getenv("MOVE", "true") == "true"

# パラメータの後処理
import numpy as np
speed_limits = np.array(speed_limits)
# 速度のソフトリミット
eff_speed_limits = speed_limits * speed_limit_ratio
accel_limits = np.array(accel_limits)
# 加速度のソフトリミット
eff_accel_limits = accel_limits * accel_limit_ratio
abs_joint_limit = np.array(abs_joint_limit)
# ロボットの関節の角度の範囲のソフトリミット (deg)
# NOTE: VR側と一致していることが望ましい
abs_joint_soft_limit = abs_joint_limit - 10
