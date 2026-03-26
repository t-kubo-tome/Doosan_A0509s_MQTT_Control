"""
ロボット固有のモジュール。
ここでロボット固有の名前のクラスを__all__でエクスポートしている共通の名前に変換してください。
"""
from robot.doosan_control_hardware import DoosanControlHardware as ControlHardware
from robot.doosan_monitor_hardware import DoosanMonitorHardware as MonitorHardware
from robot.doosan_mqtt_recv import Doosan_MQTT_Recv as MQTT_Recv

__all__ = [
    "ControlHardware",
    "MonitorHardware",
    "MQTT_Recv",
]
