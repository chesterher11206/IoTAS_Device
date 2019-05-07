import sys
import time
from client import Device

device = Device()
while True:
    try:
        time.sleep(1)
    except:
        print("System closing...")
        sys.exit(0)