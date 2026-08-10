import paho.mqtt.client as mqtt
import json
import math
import time

BROKER = '192.168.2.175'
PORT = 1883
TOPIC = 'robot/telemetry'

R = 300.0   # rayon du cercle test, en mm
DT = 0.05   # increment de t par itération, en radians

client = mqtt.Client()
client.connect(BROKER, PORT, 60)

t = 0.0

try:
    while True:
        x = R * math.cos(t)
        y = R * math.sin(t)
        theta = t + math.pi / 2

        payload = {
            "type": "TELEMETRY",
            "payload": {
                "pose": {"x": x, "y": y, "theta": theta}
            }
        }

        client.publish(TOPIC, json.dumps(payload))
        print(f"t={t:.2f} x={x:.1f} y={y:.1f} theta={theta:.2f}")

        t += DT
        time.sleep(0.1)
except KeyboardInterrupt:
    client.disconnect()
