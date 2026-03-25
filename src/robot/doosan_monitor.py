# Doosanの状態をモニタリングする
from dataclasses import dataclass

from ..common.monitor import MonitorBase, MonitorConfig
from .shared_memory import NamedSharedMemory
from .config import (
    HAND_IP,
    MQTT_ROBOT_STATE_TOPIC,
    MQTT_SERVER,
    ROBOT_IP,
    ROBOT_UUID,
    SAVE,
    joint_unit_internal,
    joint_unit_external,
)


@dataclass
class DoosanMonitorConfig(MonitorConfig):
    robot_ip: str
    hand_ip: str


class Doosan_MON(MonitorBase):
    def _get_config(self) -> DoosanMonitorConfig:
        return DoosanMonitorConfig(
            mqtt_server=MQTT_SERVER,
            mqtt_robot_state_topic=MQTT_ROBOT_STATE_TOPIC + "/" + ROBOT_UUID,
            save_state=SAVE,
            joint_unit_internal=joint_unit_internal,
            joint_unit_external=joint_unit_external,
            robot_ip=ROBOT_IP,
            hand_ip=HAND_IP,
        )

    def _get_make_shared_memory(self) -> type[NamedSharedMemory]:
        return NamedSharedMemory

    def _init_other_than_config(self) -> None:
        pass

    def format_error(self, e: Exception) -> str:
        return str(e)

    def connect_robot(self) -> None:
        pass

    def find_and_setup_hand(self, tool_id: int) -> None:
        pass

    def reconnect_robot(self) -> None:
        pass

    def disconnect_robot(self) -> None:
        pass

    def reconnect_after_timeout(self, e: Exception) -> bool:
        pass
