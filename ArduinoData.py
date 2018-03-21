import threading
import random
import time
import queue
import serial

# Hoisting contructior, read from hoistiong arduino and stores it in its own dictionary.
#It also has a method for getting the stored data in a safe matter with locks.
class HoistingData(threading.Thread):
    def __init__(self, lock):
        threading.Thread.__init__(self,daemon=True)
        self.stop_thread = False
        self.lock = lock
        self.hoistingSensor = {
            "WOB": 0,

        }
        self.hoistingQueue = queue.Queue()
        self.serialConn = serial.Serial()
    
    def setSerialPort(self,serialPort):
        self.serialConn.baudrate = 9600
        self.serialConn.port = serialPort
        self.serialConn.open()
    

    def run(self):
        while self.stop_thread != True:

            try:
                item = self.hoistingQueue.get_nowait()
                
            except:
                pass
            rand = random.randint(1, 100)
            self.lock.acquire()
            self.hoistingSensor["WOB"] = rand
            self.lock.release()
            try:
                item = self.hoistingQueue.get_nowait()
                
            except:
                pass

            time.sleep(0.05)

    def getHoistingSensorData(self):
        self.lock.acquire()
        hs = self.hoistingSensor
        self.lock.release()
        return hs

# Rotation contructior, read from rotation arduino and stores it in its own dictionary.
# It also has a method for getting the stored data in a safe matter with locks.
class RotationData(threading.Thread):
    def __init__(self, lock,rotationSensor=0):
        threading.Thread.__init__(self,daemon=True)
        self.stop_thread = False
        self.lock = lock
        self.rotationSensor = {
            "torque":0,
            "RPM":0,
            "vibration":0
        }
        self.rotationQueue = queue.Queue()
        self.serialConn = serial.Serial()

    def setSerialPort(self,serialPort):
        self.serialConn.baudrate = 9600
        self.serialConn.port = serialPort
        self.serialConn.open()

    def run(self):
        
        while self.stop_thread != True:
            
            #Get data from Arduino
            #Example on sensor data: t20
            try:
                item = self.rotationQueue.get_nowait()
                print(item)
            except:
                pass
            try:

                rotationData = self.serialConn.readline().decode().strip('\r\n')
            except:
                pass
            if rotationData:
                try:
                    sensorType = rotationData[0]
                    data = float(rotationData[1:])
                    dataPrev = data
                    if sensorType == 't':
                        self.lock.acquire()
                        self.rotationSensor["torque"] = data
                        self.lock.release()
                    if sensorType == 'r':
                        self.lock.acquire()
                        self.rotationSensor["RPM"] = data
                        self.lock.release()
                    if sensorType == 'v':
                        self.lock.acquire()
                        self.rotationSensor["vibration"] = data
                        self.lock.release()
                except:
                    pass
                    
            time.sleep(0.05)

    def getRotationSensorData(self):# Get data in a save matter with locks 
        self.lock.acquire()
        rs = self.rotationSensor
        self.lock.release()
        return rs


class CirculationData(threading.Thread):
    def __init__(self, lock):
        threading.Thread.__init__(self,daemon=True)
        self.lock = lock
        self.circulationSensor = {
            "pressure":1
        }
        self.circulationQueue = queue.Queue()
        self.serialConn = serial.Serial()
        self.stop_thread = False

    def setSerialPort(self,serialPort):
        self.serialConn.baudrate = 9600
        self.serialConn.port = serialPort
        self.serialConn.open()

    def run(self):
        while self.stop_thread != True:
            # serVal = self.serialConn.read()
            rand = random.randint(10, 20)
            self.lock.acquire()
            self.circulationSensor["pressure"] = rand
            self.lock.release()

            try:
                item = self.circulationQueue.get_nowait()
                
            except:
                pass

            time.sleep(0.05)

    def getCirculationSensorData(self):
        self.lock.acquire()
        cs = self.circulationSensor
        self.lock.release()
        return cs
    
