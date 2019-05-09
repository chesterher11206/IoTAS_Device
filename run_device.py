import sys
import time
import json
import atexit
from device import Device

def exit_handler(device):
    print("System closing...")
    device_info_json = json.dumps(device.device_info)
    if hasattr(device, 'ser'):
        device.ser.close()
    device.response_client.publish("device/disconnect/uuid", payload=device_info_json, qos=0)

device = Device()
atexit.register(exit_handler, device)
while True:
    time.sleep(1)