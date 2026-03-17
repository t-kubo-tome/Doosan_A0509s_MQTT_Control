# MQTTを受信する
import json
import logging
import logging.handlers
import os
import time
from datetime import datetime

from dotenv import load_dotenv
from paho.mqtt import client as mqtt

from ..common.utils import rad2deg_list
from .shared_memory import NamedSharedMemory

# パラメータ
load_dotenv(os.path.join(os.path.dirname(__file__),'.env'))
MQTT_SERVER = os.getenv("MQTT_SERVER", "sora2.uclab.jp")
MQTT_CTRL_TOPIC = os.getenv("MQTT_CTRL_TOPIC", "control")
ROBOT_UUID = os.getenv("ROBOT_UUID","ur-real")
ROBOT_MODEL = os.getenv("ROBOT_MODEL","ur-real")
MQTT_MANAGE_TOPIC = os.getenv("MQTT_MANAGE_TOPIC", "mgr")
MQTT_MANAGE_RCV_TOPIC = os.getenv("MQTT_MANAGE_RCV_TOPIC", "dev")+"/"+ROBOT_UUID
MQTT_COMMAND_TOPIC = os.getenv("MQTT_COMMAND_TOPIC", "dev") + "/" + ROBOT_UUID + "/command"


class MQTT_Recv:
    def __init__(self):
        self.mqtt_ctrl_topic = None
        self.last_registered = None
        self.command_queue = None

    def on_connect(self, client, userdata, connect_flags, reason_code, properties):
        # マネージャに登録
        now = time.time()
        self._register_to_manager(now)
        # ロボットへの連絡用トピックの購読
        self.client.subscribe(MQTT_MANAGE_RCV_TOPIC)
        self.logger.info("subscribe to: " + MQTT_MANAGE_RCV_TOPIC)
        # ロボットの汎用制御コマンドトピックの購読
        self.client.subscribe(MQTT_COMMAND_TOPIC)
        self.logger.info("subscribe to: " + MQTT_COMMAND_TOPIC)

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
        # 汎用制御コマンドトピックの処理
        if msg.topic == MQTT_COMMAND_TOPIC:
            self._on_mqtt_command_topic(msg)
        # リアルタイム制御トピックの処理
        elif msg.topic == self.mqtt_ctrl_topic:
            self._on_mqtt_ctrl_topic(msg)
        # ロボットへの連絡用トピックの処理
        elif msg.topic == MQTT_MANAGE_RCV_TOPIC:
            js = json.loads(msg.payload)
            goggles_id = js["devId"]
            mqtt_ctrl_topic = MQTT_CTRL_TOPIC + "/" + goggles_id
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
            "type": ROBOT_MODEL,
            "version": "none",
            "devId": ROBOT_UUID,
        }
        # マネージャに登録
        self.client.publish(MQTT_MANAGE_TOPIC + "/register", json.dumps(info))
        # 表示用
        info["topic"] = MQTT_MANAGE_TOPIC + "/register"
        self.topic_memory.write("mgr/register", dict(info))
        self.logger.info("publish to: " + MQTT_MANAGE_TOPIC + "/register")
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
        self.client.connect(MQTT_SERVER, 1883, 60)
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
        self.shm = NamedSharedMemory(create=False)
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
                info = {"devId": ROBOT_UUID}
                self.client.publish(
                    MQTT_MANAGE_TOPIC + "/unregister", json.dumps(info))
                self.logger.info(
                    "publish to: " + MQTT_MANAGE_TOPIC + "/unregister")
                self.client.loop_stop()
                self.client.disconnect()
                self.shm.release()
                time.sleep(1)
                self.logger.info("Process stopped")
                self.handler.close()
                break

            time.sleep(1)

    def _on_mqtt_ctrl_topic(self, msg):
        # ロボット固有の実装は基本的にここだけで完結するはず
        js = json.loads(msg.payload)
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

        self.shm.is_joint_target_received = 1
        js["topic"] = msg.topic
        self.topic_memory.write("control", dict(js))
