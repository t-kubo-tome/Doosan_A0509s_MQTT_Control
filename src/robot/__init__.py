"""
ロボット固有のモジュール。
ここでロボット固有の名前のクラスを__all__でエクスポートしている共通の名前に変換してください。
"""
from robot.doosan_control import Doosan_CON as CON
from robot.doosan_monitor import Doosan_MON as MON
from robot.doosan_mqtt_recv import Doosan_MQTT_Recv as MQTT_Recv
from robot.doosan_shared_memory import DoosanNamedSharedMemory as NamedSharedMemory

__all__ = [
    "CON",
    "MON",
    "MQTT_Recv",
    "NamedSharedMemory",
]
