from dataclasses import dataclass


@dataclass
class MQTTConfig:
    mqtt_server: str
    robot_uuid: str
    robot_model: str
    mqtt_ctrl_topic: str
    mqtt_manage_topic: str
    mqtt_robot_state_topic: str
