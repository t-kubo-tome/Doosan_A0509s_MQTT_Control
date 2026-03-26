from typing import Any

from common.mqtt_recv_base import MQTT_Recv_Base


class Doosan_MQTT_Recv(MQTT_Recv_Base):
    def on_init(self) -> None:
        """追加の初期化処理。"""
        pass

    def _interpret_mqtt_ctrl_topic(self, js: dict[str, Any]) -> None:
        """
        MQTT制御トピックの内容を解釈して共有メモリに反映。
        この実装内で、js中の関節角度(例: joints)が存在する場合、
        必ずself._angle_unit_converter.to_internal(joints)
        で角度を内部単位に変換したうえで共有メモリに反映すること。
        """
        if "joints" in js:
            self.shm.joint_target = self._angle_unit_converter.to_internal(
                js["joints"])

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
