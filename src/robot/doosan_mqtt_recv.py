from dataclasses import dataclass
from typing import Any

from ..common.mqtt_recv import MQTT_Recv_Base, MQTTConfig
from ..common.utils import rad2deg_list
from .config import (
    MQTT_CTRL_TOPIC,
    MQTT_MANAGE_TOPIC,
    MQTT_ROBOT_STATE_TOPIC,
    MQTT_SERVER,
    ROBOT_MODEL_ENV,
    ROBOT_UUID,
)


@dataclass
class DoosanMQTTConfig(MQTTConfig):
    pass


class Doosan_MQTT_Recv(MQTT_Recv_Base):
    def _get_config(self) -> DoosanMQTTConfig:
        return DoosanMQTTConfig(
            mqtt_server=MQTT_SERVER,
            robot_uuid=ROBOT_UUID,
            robot_model=ROBOT_MODEL_ENV,
            mqtt_ctrl_topic=MQTT_CTRL_TOPIC,
            mqtt_manage_topic=MQTT_MANAGE_TOPIC,
            mqtt_robot_state_topic=MQTT_ROBOT_STATE_TOPIC,
        )

    def _init_other_than_config(self) -> None:
        pass

    def _interpret_mqtt_ctrl_topic(self, js: dict[str, Any]) -> None:
        if "joints" in js:
            self.shm.joint_target = rad2deg_list(js["joints"])

        if "grip" in js:
            right_grip = js['grip'][1]
            if right_grip:
                self.shm.hand_target = 1
            else:
                self.shm.hand_target = 2

        if "tool_change" in js:
            if self.shm.tool_change == 0:
                tool = js["tool_change"]
                self.shm.stop_realtime_control = 1
                self.shm.tool_change = tool

        if "put_down_box" in js:
            if self.shm.demo_put_down_box == 0:
                if js["put_down_box"]:
                    self.shm.stop_realtime_control = 1
                    self.shm.demo_put_down_box = 1

        if "line_cut" in js:
            if self.shm.line_cut == 0:
                if js["line_cut"]:
                    self.shm.stop_realtime_control = 1
                    self.shm.line_cut = 1
