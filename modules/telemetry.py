import paho.mqtt.client as mqtt
import json
import config

class MqttHandler:
    def __init__(self):
        self.client = mqtt.Client(client_id="DronePyBullet")
        self.connected = False
        
        try:
            self.client.connect(config.MQTT_BROKER, config.MQTT_PORT, 60)
            self.client.loop_start() # Roda em thread separada
            self.connected = True
            print("[MQTT] Conectado ao Broker.")
        except Exception as e:
            print(f"[MQTT] Erro ao conectar: {e}")

    def send_data(self, state, current_pos, target_pos, pending_count):
        if not self.connected:
            return

        payload = {
            "state": state,
            "position": {"x": round(current_pos[0], 2), "y": round(current_pos[1], 2)},
            "current_target": target_pos,
            "pending_targets": pending_count
        }
        self.client.publish(config.MQTT_TOPIC_TELEMETRY, json.dumps(payload))