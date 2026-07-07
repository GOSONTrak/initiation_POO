from os import getenv
from dotenv import load_dotenv
from pathlib import Path
from influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import SYNCHRONOUS
from influxdb_client import Point
import time
import paho.mqtt.client as mqtt
import json
import os

if load_dotenv(Path(__file__).parent / ".env"):

    INFLUXDB_URL = os.getenv("INFLUXDB_URL")
    INFLUXDB_ORG = os.getenv("INFLUXDB_ORG")
    INFLUXDB_BUCKET = os.getenv("INFLUXDB_BUCKET")
    INFLUXDB_TOKEN = os.getenv("INFLUXDB_TOKEN")

    BROKER = '10.101.231.191'
    PORT   = 1883
    TOPIC = 'robot/telemetry'
    last_tag = ''


    client = InfluxDBClient(url=INFLUXDB_URL, token=INFLUXDB_TOKEN, org=INFLUXDB_ORG)
    write_api = client.write_api(write_options=SYNCHRONOUS)


    def handle_telemetry(payload):

        points = []

        # Timestamp ESP32 (optionnel)
        ts =  payload["ts"]

        # Batterie
        b = payload["battery"]
        points.append(
            Point("battery")
            .field("voltage", b["voltage"])
            .field("percent", b["percent"])
            .field("critical", b["critical"])

        )

        # Capteurs de ligne
        line = payload["line"]
        points.append(
            Point("line")
            .field("front_left", line["frontLeft"])
            .field("front_right", line["frontRight"])
            .field("back_left", line["backLeft"])
            .field("back", line["back"])
        )

        # ToF
        tof = payload["tof"]
        points.append(
            Point("tof")
            .field("front_left", tof["frontLeft"])
            .field("front_right", tof["frontRight"])
        )

        # Lidar
        lidar = payload["lidar"]
        points.append(
            Point("lidar")
            .field("distance", lidar["dist"])
            .field("angle", lidar["angle"])
            .field("valid", lidar["valid"])
        )

        # IMU
        imu = payload["imu"]
        points.append(
            Point("imu")
            .field("ax", imu["ax"])
            .field("ay", imu["ay"])
            .field("az", imu["az"])
            .field("gx", imu["gx"])
            .field("gy", imu["gy"])
            .field("gz", imu["gz"])
            .field("valid", imu["valid"])
        )

        # Moteurs
        motors = payload["motors"]
        points.append(
            Point("motors")
            .field("left_speed", motors["left"])
            .field("right_speed", motors["right"])
        )

        # Encodeurs
        enc = payload["encoders"]
        points.append(
            Point("encoders")
            .field("left_rpm", enc["leftRpm"])
            .field("right_rpm", enc["rightRpm"])
        )

        # Position estimée
        pose = payload["pose"]
        points.append(
            Point("pose")
            .field("x", pose["x"])
            .field("y", pose["y"])
            .field("theta", pose["theta"])

        )

        write_api.write(
            bucket=INFLUXDB_BUCKET,
            org=INFLUXDB_ORG,
            record=points
        )



    def on_connect(client, userdata, flags, rc):
        print(f'[MQTT] connecte {BROKER} (rc={rc})')
        client.subscribe(TOPIC)

    def on_message(client, userdata, msg):
        global last_tag
        try:
            data = json.loads(msg.payload)
            if data.get('type') != 'TELEMETRY':
                return
            p  = data['payload']
            ml = p['motors']['left']
            mr = p['motors']['right']
            ld = p['lidar']['dist']
            la = p['lidar']['angle']
            lv = p['lidar']['valid']
            lrpm = p['encoders']['leftRpm']
            rrpm = p['encoders']['rightRpm']
            ts   = p['ts']

            if ml == 0 and mr == 0:
                tag = 'SEARCH/WAIT'
            elif ml > 0 and mr < 0:
                tag = 'ROTATE'
            elif ml < 0 and mr > 0:
                tag = 'ROTATE_INV'
            elif ml > 0 and mr > 0:
                tag = 'ATTACK'
            else:
                tag = 'MOT({},{})'.format(ml, mr)

            marker = ' <<< CHANGEMENT' if tag != last_tag else ''
            last_tag = tag

            lidar_str = 'OK' if lv else '--'
            print('[{:8d}] {:<12} MOT L={:4d} R={:4d}  LIDAR={:.2f}m {:5.1f}deg [{}]  ENC L={:6.1f} R={:6.1f}{}'.format(
                ts, tag, ml, mr, ld, la, lidar_str, lrpm, rrpm, marker))
            try:
                handle_telemetry(p)
            except Exception as e:
                print('[MQTT] {}'.format(e))

        except Exception as e:
            print('[ERR] {}'.format(e))

mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(BROKER, PORT, 60)
mqtt_client.loop_forever()



