import threading
import random
import time
import queue
import serial

# Hoisting contructior, read from hoistiong arduino and stores it in its own dictionary.
#It also has a method for getting the stored data in a safe matter with locks.
class HoistingData(threading.Thread):
    def __init__(self, lock):
        threading.Thread.__init__(self)
        self.lock = lock
        self.hoistingSensor = {
            "WOB": 0,

        }
        self.hoistingQueue = queue.Queue()
        self.serialConn = serial.Serial()

    def run(self):
        while True:
            #serVal = self.serialConn.read()
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
        threading.Thread.__init__(self)
        self.lock = lock
        self.rotationSensor = {
            "torque":0,
            "RPM":0,
            "vibration":2
        }
        self.rotationQueue = queue.Queue()
        self.serialConn = serial.Serial(port='/dev/cu.usbmodem1431',baudrate= 9600)

    def run(self):
        dataPrev = 0
        while True:
            #Get data from Arduino
            #Example on sensor data: t20
            rotationData = self.serialConn.readline().decode().strip('\r\n')
            if not rotationData:
                continue

            sensorType = rotationData[0]
            try:
                data = float(rotationData[1:])
                dataPrev = data
            except:
                data = dataPrev
            rand = random.randint(5, 10)
            if sensorType == 't':
                self.lock.acquire()
                self.rotationSensor["torque"] = data
                self.lock.release()
            elif sensorType == 'r':
                self.lock.acquire()
                self.rotationSensor["RPM"] = data
                self.lock.release()
            elif sensorType == 'v':
                self.lock.acquire()
                self.rotationSensor["vibration"] = data
                self.lock.release()
            else:
                pass
            try:
                item = self.rotationQueue.get_nowait()
                
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
        threading.Thread.__init__(self)
        self.lock = lock
        self.circulationSensor = {
            "pressure":1
        }
        self.circulationQueue = queue.Queue()
        self.serialConn = serial.Serial()

    def run(self):
        while True:
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
