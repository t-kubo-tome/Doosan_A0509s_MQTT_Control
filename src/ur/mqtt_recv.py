# MQTTを受信する


import json
import logging
from paho.mqtt import client as mqtt
import multiprocessing.shared_memory


import os
from datetime import datetime
import numpy as np
import time
import sys

## ここでUUID を使いたい
import uuid

package_dir = os.path.abspath(os.path.dirname(__file__))
sys.path.append(package_dir)
from config import SHM_NAME, SHM_SIZE

from dotenv import load_dotenv

# パラメータ
load_dotenv(os.path.join(os.path.dirname(__file__),'.env'))
MQTT_SERVER = os.getenv("MQTT_SERVER", "sora2.uclab.jp")
MQTT_CTRL_TOPIC = os.getenv("MQTT_CTRL_TOPIC", "control")
ROBOT_UUID = os.getenv("ROBOT_UUID","ur-real")
ROBOT_MODEL = os.getenv("ROBOT_MODEL","ur-real")
MQTT_MANAGE_TOPIC = os.getenv("MQTT_MANAGE_TOPIC", "mgr")
MQTT_MANAGE_RCV_TOPIC = os.getenv("MQTT_MANAGE_RCV_TOPIC", "dev")+"/"+ROBOT_UUID
MQTT_MODE = os.getenv("MQTT_MODE", "metawork")


class MQTT_Recv:
    def __init__(self):
        self.mqtt_ctrl_topic = None
        self.last_registered = None
 
    def on_connect(self, client, userdata, connect_flags, reason_code, properties):
        # ロボットのメタ情報の中身はとりあえず
        date = datetime.now().strftime('%c')
        if MQTT_MODE == "metawork":
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
            self.client.publish(MQTT_MANAGE_TOPIC + "/register", json.dumps(info))
            with self.mqtt_control_lock:
                info["topic_type"] = "mgr/register"
                info["topic"] = MQTT_MANAGE_TOPIC + "/register"
                self.mqtt_control_dict.clear()
                self.mqtt_control_dict.update(info)
            self.logger.info("publish to: " + MQTT_MANAGE_TOPIC + "/register")
            self.last_registered = time.time()
            self.client.subscribe(MQTT_MANAGE_RCV_TOPIC)
            self.logger.info("subscribe to: " + MQTT_MANAGE_RCV_TOPIC)
        else:
            self.logger.info("MQTT:Connected with result code " + str(rc),
                             "subscribe ctrl", MQTT_CTRL_TOPIC)
            self.mqtt_ctrl_topic = MQTT_CTRL_TOPIC
            self.client.subscribe(self.mqtt_ctrl_topic)

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
        if msg.topic == self.mqtt_ctrl_topic:
            js = json.loads(msg.payload)

            joints=['j1','j2','j3','j4','j5','j6']
            rot =[js[x]  for x in joints]    
            joint_q = [x for x in rot]
            self.pose[6:12] = joint_q 

            if "grip" in js:
                if js['grip']:
                    self.pose[13] = 1
                else:
                    self.pose[13] = 2
            
            if "tool_change" in js:
                if self.pose[17] == 0:
                    tool = js["tool_change"]
                    self.pose[16] = 1
                    self.pose[17] = tool
            
            if "put_down_box" in js:
                if self.pose[21] == 0:
                    if js["put_down_box"]:
                        self.pose[16] = 1
                        self.pose[21] = 1
            
            if "line_cut" in js:
                if self.pose[38] == 0:
                    if js["line_cut"]:
                        self.pose[16] = 1
                        self.pose[38] = 1

            self.pose[20] = 1
            with self.mqtt_control_lock:
                js["topic_type"] = "control"
                js["topic"] = msg.topic
                self.mqtt_control_dict.clear()
                self.mqtt_control_dict.update(js)

        elif msg.topic == MQTT_MANAGE_RCV_TOPIC:
            if MQTT_MODE == "metawork":
                js = json.loads(msg.payload)
                goggles_id = js["devId"]
                mqtt_ctrl_topic = MQTT_CTRL_TOPIC + "/" + goggles_id
                if mqtt_ctrl_topic != self.mqtt_ctrl_topic:
                    if self.mqtt_ctrl_topic is not None:
                        self.client.unsubscribe(self.mqtt_ctrl_topic)    
                    self.mqtt_ctrl_topic = mqtt_ctrl_topic
                self.client.subscribe(self.mqtt_ctrl_topic)
                self.logger.info("subscribe to: " + self.mqtt_ctrl_topic)
                with self.mqtt_control_lock:
                    js["topic_type"] = "dev"
                    js["topic"] = msg.topic
                    self.mqtt_control_dict.clear()
                    self.mqtt_control_dict.update(js)
        else:
            self.logger.warning("not subscribe msg" + msg.topic)

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

    def run_proc(self, mqtt_control_dict, mqtt_control_lock, log_queue):
        self.setup_logger(log_queue)
        self.logger.info("Process started")
        self.sm = multiprocessing.shared_memory.SharedMemory(SHM_NAME)
        self.pose = np.ndarray((SHM_SIZE,), dtype=np.dtype("float32"), buffer=self.sm.buf)
        self.mqtt_control_dict = mqtt_control_dict
        self.mqtt_control_lock = mqtt_control_lock
        self.connect_mqtt()
        while True:
            # 30分ごとに再登録
            now = time.time()
            if (self.last_registered is not None and 
                self.last_registered + 60 * 30 < now):
                date = datetime.now().strftime('%c')
                if MQTT_MODE == "metawork":
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
                    self.client.publish(
                        MQTT_MANAGE_TOPIC + "/register", json.dumps(info))
                    with self.mqtt_control_lock:
                        info["topic_type"] = "mgr/register"
                        info["topic"] = MQTT_MANAGE_TOPIC + "/register"
                        self.mqtt_control_dict.clear()
                        self.mqtt_control_dict.update(info)
                    self.logger.info(
                        "re-publish to: " + MQTT_MANAGE_TOPIC + "/register")
                    self.last_registered = now

            # プロセス終了時
            if self.pose[32] == 1:
                if MQTT_MODE == "metawork":
                    info = {"devId": ROBOT_UUID}
                    self.client.publish(
                        MQTT_MANAGE_TOPIC + "/unregister", json.dumps(info))
                    self.logger.info(
                        "publish to: " + MQTT_MANAGE_TOPIC + "/unregister")
                self.client.loop_stop()
                self.client.disconnect()
                self.sm.close()
                time.sleep(1)
                self.logger.info("Process stopped")
                self.handler.close()
                break

            time.sleep(1)
