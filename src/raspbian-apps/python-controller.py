import time, serial, threading, queue, logging
from datetime import datetime
logging.basicConfig(level=logging.DEBUG);
# from platform import system
# system_name = system()
# if system_name == 'Windows':
#     import serial.serialwin32 as serial
# elif system_name == 'Linux':
#     import serial.serialposix as serial
from serial import Serial
import psycopg2

conn = psycopg2.connect(host="192.168.0.196", database="greenhouse", user="postgres", password="postgres")


q = queue.Queue(10)
portName = "/dev/ttyUSB0"
logging.info("Connecting to serial port %s", portName)
try:
    comPort = Serial(portName, 9600, timeout=1)
    logging.info("Connected to %s", portName)
except Exception as e:
    comPort = None
    logging.warning("Not connected: ", e)

class ControllerConfig:
    def __init__(self, port, prefix, setpoint = 0, deviation = 1, k = 0.1):
        logging.info("Creating controller config with prefix %s", prefix)
        self.port = port
        self.prefix = prefix
        self.setpoint = setpoint
        self.deviation = deviation
        self.k = k
    def sendSettings(self):
        q.put(f"{self.prefix};SP={self.setpoint}")
        q.put(f"{self.prefix};D={self.deviation}")
        q.put(f"{self.prefix};K={self.k}");

airConfig = ControllerConfig(comPort, "A", 24, 0.5)
humidityConfig = ControllerConfig(comPort, "H", 90, 1)
co2Config = ControllerConfig(comPort, "C", 5000, 100)
airConfig.sendSettings()
humidityConfig.sendSettings()
co2Config.sendSettings()

receiveFlag = True
def Transmit():
    while True:
        item = q.get()
        if (item == None):
            logging.info("Quitting...")
            return
        logging.info("Sending command %s", item)
        strItem = str(item).encode("utf-8")
        comPort.write(strItem)

def Receive():
    while receiveFlag:
        if (comPort.in_waiting > 0):
            line = comPort.readline().decode('utf-8').rstrip()
            values = line.split(' ')
            try:
                air = float(values[0].split(':')[1])
                compost  = float(values[1].split(':')[1])
                humidity = float(values[2].split(':')[1])
                co2 = int(values[3].split(':')[1])
                time = datetime.now()

                sql = f"""INSERT INTO public."data" ("time", air, compost, co2, humidity) VALUES(%s, %s, %s, %s, %s);"""

                cur = conn.cursor()
                cur.execute(sql, (time, air, compost, co2, humidity))
                conn.commit()
                cur.close()
            except Exception as e:
                logging.warning('Receive error %s', e)

            

threading.Thread(target=Transmit).start()
threading.Thread(target=Receive).start()

try:
    while True:
        command = input('Command:').rstrip()
        if ("exit" in command):
            q.put(None)
            break
        else:
            q.put(command)
except:
    q.put(None)


        
