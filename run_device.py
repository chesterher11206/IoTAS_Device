import sys
import time
from device import Device

device = Device()
while True:
    try:
        time.sleep(1)
    except:
        print("System closing...")
        sys.exit(0)