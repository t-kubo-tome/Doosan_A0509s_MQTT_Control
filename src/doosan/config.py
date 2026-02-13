SHM_NAME = "doosan"
SHM_SIZE = 64
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
