# #!/usr/bin/env python3
import serial
import time
from datetime import datetime
import threading

serial_busy = False
ser = None

def get_data_from_serial():
    if ser != None and not serial_busy:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            print(f"{datetime.date()}: {line}")
    else:
        time.sleep(0.1)

if __name__ == "__main__":
    ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)
    ser.flush()
    read_thread = threading.Thread(target=get_data_from_serial, args=())
    read_thread.daemon = True
    print('Press Enter to stop')
    read_thread.start()
    value = input()


    