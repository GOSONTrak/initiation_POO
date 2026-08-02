from logging import critical
from os import getenv
from dotenv import load_dotenv
from pathlib import Path
from influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import SYNCHRONOUS
from influxdb_client import Point
from math import sin, cos, pi, radians
import time
import math
import random
import os

if load_dotenv(Path(__file__).parent / ".env"):

    INFLUXDB_URL = os.getenv("INFLUXDB_URL")
    INFLUXDB_ORG = os.getenv("INFLUXDB_ORG")
    INFLUXDB_BUCKET = os.getenv("INFLUXDB_BUCKET")
    INFLUXDB_TOKEN = os.getenv("INFLUXDB_TOKEN")

    client = InfluxDBClient(url=INFLUXDB_URL, token=INFLUXDB_TOKEN, org=INFLUXDB_ORG)
    write_api = client.write_api(write_options=SYNCHRONOUS)
if __name__ == '__main__':

    for i in range(60):
        value_bat =  8.4 - (i * 3.7/60)
        percent_bat= value_bat * 100/8.4
        criticals = False
        if percent_bat < 20.0:
            criticals = True
        point = (Point("battery")
        .field("voltage", value_bat)
        .field("percent", percent_bat)
        .field("critical",criticals)
        .field("ps", time.time()))
        time.sleep(5)
        write_api.write(bucket=INFLUXDB_BUCKET, record= point, org=INFLUXDB_ORG)
        print(value_bat, percent_bat, criticals)

        poin_m = Point("speed motor")
        poin_m.field("speed_left", random.randint(-255, 255))
        poin_m.field("speed_rigth", random.randint(-255, 255))
        poin_m.field("ps", time.time())
        write_api.write(bucket= INFLUXDB_BUCKET, record= poin_m, org=INFLUXDB_ORG)

        poin = Point("position EKF")
        poin.field("X", random.randint(-255, 255))
        poin.field("Y", random.randint(-255, 255))
        poin.field("Theta", random.randint(-pi.__int__(), pi.__int__()))
        poin.field("ps", time.time())
        write_api.write(bucket=INFLUXDB_BUCKET, record=poin, org=INFLUXDB_ORG)