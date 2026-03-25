# MQTTを受信する
import json
import logging
import logging.handlers
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from datetime import datetime

from paho.mqtt import client as mqtt

from .shared_memory_base import NamedSharedMemoryBase


@dataclass
class MQTTConfig:
    mqtt_server: str
    robot_uuid: str
    robot_model: str
    mqtt_ctrl_topic: str
    mqtt_manage_topic: str
    mqtt_robot_state_topic: str
    joint_unit_internal: str
    joint_unit_external: str


class MQTT_Recv_Base(ABC):
    """MQTTを受信して共有メモリに書き込むプロセスの基底クラス."""
    @abstractmethod
    def _get_config(self) -> MQTTConfig:
        pass

    @abstractmethod
    def _init_other_than_config(self) -> None:
        pass

    @abstractmethod
    def _get_make_shared_memory(self) -> type[NamedSharedMemoryBase]:
        pass

    @abstractmethod
    def _interpret_mqtt_ctrl_topic(self, js) -> None:
        """
        MQTT制御トピックの内容を解釈して共有メモリに反映。
        この実装内で、js中の関節角度(例: joints)が存在する場合、
        必ずself._angle_unit_converter.to_internal(joints)
        で角度を内部単位に変換したうえで共有メモリに反映すること。
        """
        pass

    def __init__(self):
        self.config = self._get_config()
        self._init_other_than_config()
        self.mqtt_ctrl_topic = None
        self.last_registered = None
        self.command_queue = None

    def on_connect(self, client, userdata, connect_flags, reason_code, properties):
        # マネージャに登録
        now = time.time()
        self._register_to_manager(now)
        # ロボットへの連絡用トピックの購読
        mqtt_manage_rcv_topic = "dev/" + self.config.robot_uuid
        self.client.subscribe(mqtt_manage_rcv_topic)
        self.logger.info("subscribe to: " + mqtt_manage_rcv_topic)
        # ロボットの汎用制御コマンドトピックの購読
        mqtt_command_topic = "dev/" + self.config.robot_uuid + "/command"
        self.client.subscribe(mqtt_command_topic)
        self.logger.info("subscribe to: " + mqtt_command_topic)

    def on_disconnect(
        self,
        client,
        userdata,
        disconnect_flags,
        reason_code,
        properties,
    ):
        if reason_code != 0:
            self.logger.warning("MQTT Unexpected disconnection.")

    def on_message(self, client, userdata, msg):
        mqtt_manage_rcv_topic = "dev/" + self.config.robot_uuid
        mqtt_command_topic = "dev/" + self.config.robot_uuid + "/command"
        # 汎用制御コマンドトピックの処理
        if msg.topic == mqtt_command_topic:
            self._on_mqtt_command_topic(msg)
        # リアルタイム制御トピックの処理
        elif msg.topic == self.mqtt_ctrl_topic:
            self._on_mqtt_ctrl_topic(msg)
        # ロボットへの連絡用トピックの処理
        elif msg.topic == mqtt_manage_rcv_topic:
            js = json.loads(msg.payload)
            goggles_id = js["devId"]
            mqtt_ctrl_topic = self.config.mqtt_ctrl_topic + "/" + goggles_id
            # VRコントローラに始めて接続する場合か既に異なるVRコントローラに接続されている場合
            if mqtt_ctrl_topic != self.mqtt_ctrl_topic:
                # 既に異なるVRコントローラに接続されている場合はその接続を解除
                if self.mqtt_ctrl_topic is not None:
                    self.client.unsubscribe(self.mqtt_ctrl_topic)
                self.mqtt_ctrl_topic = mqtt_ctrl_topic
                self.client.subscribe(mqtt_ctrl_topic)
                self.logger.info("subscribe to: " + mqtt_ctrl_topic)
                # 表示用
                js["topic"] = msg.topic
                self.topic_memory.write("dev", dict(js))
        # ロボットへの連絡用トピックの処理で接続を切り替えた直後に到達することがありうる
        else:
            self.logger.warning("Not subscribing topic: " + msg.topic)

    def _register_to_manager(self, now: float) -> None:
        # ロボットのメタ情報の中身はとりあえず
        date = datetime.now().strftime('%c')
        info = {
            "date": date,
            "device": {
                "agent": "none",
                "cookie": "none",
            },
            "devType": "robot",
            "type": self.config.robot_model,
            "version": "none",
            "devId": self.config.robot_uuid,
        }
        # マネージャに登録
        self.client.publish(self.config.mqtt_manage_topic + "/register", json.dumps(info))
        # 表示用
        info["topic"] = self.config.mqtt_manage_topic + "/register"
        self.topic_memory.write("mgr/register", dict(info))
        self.logger.info("publish to: " + self.config.mqtt_manage_topic + "/register")
        # 定期的な再登録用に時間を記録
        self.last_registered = now

    def _on_mqtt_command_topic(self, msg):
        """汎用制御コマンドを処理してキューに追加"""
        # コマンドはユーザーが作成するため不正なコマンドが来る可能性があるため例外処理を行う
        try:
            js = json.loads(msg.payload)
            if "command" not in js:
                self.logger.warning("Invalid command: missing 'command' key")
                return
            if self.command_queue is None:
                self.logger.warning("Command queue not initialized")
                return
            self.command_queue.put(js)
        except Exception:
            self.logger.error("Failed to handle command", exc_info=True)

    def connect_mqtt(self):
        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        # MQTTの接続設定
        self.client.on_connect = self.on_connect         # 接続時のコールバック関数を登録
        self.client.on_disconnect = self.on_disconnect   # 切断時のコールバックを登録
        self.client.on_message = self.on_message         # メッセージ到着時のコールバック
        self.client.connect(self.config.mqtt_server, 1883, 60)
        self.client.loop_start()   # 通信処理開始

    def setup_logger(self, log_queue):
        self.logger = logging.getLogger("MQTT")
        if log_queue is not None:
            self.handler = logging.handlers.QueueHandler(log_queue)
        else:
            self.handler = logging.StreamHandler()
        self.logger.addHandler(self.handler)
        self.logger.setLevel(logging.INFO)

    def run_proc(self, topic_memory, log_queue, command_queue=None):
        self.setup_logger(log_queue)
        self.logger.info("Process started")
        self.shm = self._get_make_shared_memory()(create=False)
        self.topic_memory = topic_memory
        self.command_queue = command_queue
        self.connect_mqtt()
        while True:
            # 30分ごとに再登録
            now = time.time()
            if (self.last_registered is not None and
                    self.last_registered + 60 * 30 < now):
                self._register_to_manager(now)

            # プロセス終了時
            if self.shm.exit_program == 1:
                info = {"devId": self.config.robot_uuid}
                self.client.publish(
                    self.config.mqtt_manage_topic + "/unregister", json.dumps(info))
                self.logger.info(
                    "publish to: " + self.config.mqtt_manage_topic + "/unregister")
                self.client.loop_stop()
                self.client.disconnect()
                self.shm.release()
                time.sleep(1)
                self.logger.info("Process stopped")
                self.handler.close()
                break

            time.sleep(1)

    def _on_mqtt_ctrl_topic(self, msg):
        js = json.loads(msg.payload)
        self._interpret_mqtt_ctrl_topic(js)
        self.shm.is_joint_target_received = 1
        js["topic"] = msg.topic
        self.topic_memory.write("control", js)
