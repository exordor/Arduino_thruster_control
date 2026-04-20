#!/usr/bin/env python3
import argparse
import json
import time

import paho.mqtt.client as mqtt


TOPICS = [
    "arduino/system/online",
    "arduino/thruster/status",
    "arduino/flow/status",
    "arduino/dht/status",
]


def on_connect(client, userdata, flags, reason_code, properties=None):
    print(f"connected: rc={reason_code}")
    for topic in TOPICS:
        client.subscribe(topic)


def on_message(client, userdata, msg):
    print(f"{msg.topic}: {msg.payload.decode()}")


def publish_control(client, left_us, right_us):
    payload = {"left_us": left_us, "right_us": right_us, "seq": int(time.time() * 1000)}
    client.publish("arduino/thruster/cmd", json.dumps(payload), qos=0, retain=False)


def publish_lease(client):
    payload = {"client": "mqtt-test", "ts_ms": int(time.time() * 1000)}
    client.publish("arduino/thruster/lease", json.dumps(payload), qos=0, retain=False)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.50.200")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--left", type=int, default=1500)
    parser.add_argument("--right", type=int, default=1500)
    parser.add_argument("--publish", action="store_true")
    parser.add_argument("--lease-only", action="store_true")
    args = parser.parse_args()

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="mqtt-test-helper")
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(args.host, args.port, 60)
    client.loop_start()

    try:
        while True:
            if args.publish or args.lease_only:
                publish_lease(client)
            if args.publish and not args.lease_only:
                publish_control(client, args.left, args.right)
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
