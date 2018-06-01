import threading
import random
import time
import queue
import serial
import logging,sys
from time import sleep
import database
import Classifier


logging.basicConfig(stream=sys.stderr, level= logging.DEBUG, datefmt='%Y-%m-%d %H:%M:%S')

# Hoisting contructior, read from hoistiong arduino and stores it in its own dictionary.
#It also has a method for getting the stored data in a safe matter with locks.
class HoistingData(threading.Thread):
    def __init__(self, lock):
        threading.Thread.__init__(self,daemon=True)
        self.stop_thread = False
        self.lock = lock
        self.hoistingSensor = {
            "brakeStatus" : 0,
            "x1" : 0,
            "x2" : 0,
            "x3" : 0,
            "y1" : 0,
            "y2" : 0,
            "y3" : 0,
            "z1" : 0,
            "z2" : 0,
            "z3" : 0,
            "sumX" : 0,
            "sumY" : 0,
            "sumZ" : 0,
            "stepperArduinoPos1" : 0,
            "stepperArduinoPos2" : 0,
            "stepperArduinoPos3" : 0,
            "hoistingMode" : 0,
            "heightSensor" : 0,
            "rop" : [],
            "rop3m" : [],
            "wob" : 0,
            "TVD" : 0
            
        }
        self.hoistingQueue = queue.Queue()
        self.serialConn = serial.Serial()

        self.hookLoad = 0
        self.WOBSetPoint = 0
        self.taggedBottom = False
        self.newTVD = 0
        self.oldTVD = 0

    
    def setSerialPort(self,serialPort):
        try:

            self.serialConn.baudrate = 115200
            self.serialConn.port = serialPort
            self.serialConn.write_timeout = 0
            #self.serialConn.set_buffer_size(rx_size = 128000, tx_size = 128000)
            self.serialConn.open()
        except:
            logging.debug("Com port already in use")
    

    def run(self):
        old = time.time()
        self.serialConn.flushInput()
        while self.stop_thread != True:

            if not self.hoistingQueue.empty():
                #self.serialConn.reset_input_buffer()
                #self.serialConn.reset_output_buffer()
                item = self.hoistingQueue.get_nowait()
                self.serialConn.write(item.encode())
           
            try:
                #self.serialConn.reset_input_buffer()
                hoistingData = self.serialConn.readline().decode().strip('\r\n').split("y")
                
            except:
                logging.debug("Cant recive hoisting arduino data ")
                hoistingData = None
                pass
            if hoistingData:
                try:
                    self.lock.acquire()
                    self.hoistingSensor["hoistingMode"] = float(hoistingData[0].split("x")[1])
                    self.hoistingSensor["brakeStatus"] = float(hoistingData[2])
                    self.hoistingSensor["heightSensor"] = float(hoistingData[3])
                    self.hoistingSensor["stepperArduinoPos1"] = float(hoistingData[4])
                    self.hoistingSensor["stepperArduinoPos2"] = float(hoistingData[5])
                    self.hoistingSensor["stepperArduinoPos3"] = float(hoistingData[6])
                    self.hoistingSensor["z1"] = 4.376*(float(hoistingData[13])) -7343.673
                    self.hoistingSensor["z2"] =  4.373*(float(hoistingData[14])) -7338.097
                    self.hoistingSensor["z3"] =  4.363*(float(hoistingData[15])) -7321.419
                    self.hoistingSensor["sumZ"] = ((4.376*(float(hoistingData[13])) -7343.673) + (4.373*(float(hoistingData[14])) -7338.097) + (4.363*(float(hoistingData[15])) -7321.419))/1000
                   
                    if time.time() - old <= 15:
                        self.hoistingSensor["rop"].append(float(hoistingData[4]))
                    else:
                        self.hoistingSensor["rop"].pop(0)
                        self.hoistingSensor["rop"].append(float(hoistingData[4]))
                   
                    if time.time() - old <= 180:
                        self.hoistingSensor["rop3m"].append(float(hoistingData[4]))
                    else:
                        self.hoistingSensor["rop3m"].pop(0)
                        self.hoistingSensor["rop3m"].append(float(hoistingData[4]))

                    self.hoistingSensor["wob"] = self.hookLoad - ((4.376*(float(hoistingData[13])) -7343.673) + (4.373*(float(hoistingData[14])) -7338.097) + (4.363*(float(hoistingData[15])) -7321.419))
                    if self.taggedBottom == False and self.hoistingSensor["wob"] >= self.WOBSetPoint:
                        self.taggedBottom == True

                    
                    self.newTVD = (self.hoistingSensor["stepperArduinoPos1"] - self.oldTVD)

                    self.hoistingSensor["TVD"] = self.newTVD
                    database.new_inc_data = True
                    Classifier.new_inc_data = True
                    self.lock.release()
                    
                except:
                    try:
                        self.lock.release()
                    except:
                        pass
                    logging.debug("Cant place hoisting arduino data in dictonary")
                    print(hoistingData)
                    pass
         
    def resetTVD(self):
        self.oldTVD = self.getHoistingSensorData()["stepperArduinoPos1"]
        
    def getHoistingSensorData(self):
        self.lock.acquire()
        hs = self.hoistingSensor
        self.lock.release()
        return hs

    def resetHookload(self):
        self.lock.acquire()
        self.hookLoad = self.hoistingSensor["sumZ"]
        self.lock.release()
# Rotation contructior, read from rotation arduino and stores it in its own dictionary.
# It also has a method for getting the stored data in a safe matter with locks.
class RotationData(threading.Thread):
    def __init__(self, lock,rotationSensor=0):
        threading.Thread.__init__(self,daemon=True)
        self.stop_thread = False
        self.lock = lock
        self.rotationSensor = {
            "topDriveMode":0,
            "measuredRPM":0,
            "torqueMotor":0,
            "torqueSensor":0
        }
        self.rotationQueue = queue.Queue()
        self.serialConn = serial.Serial()

        self.overTorqueCounter = 0
        self.motorOverLoadCounter = 0

    def setSerialPort(self,serialPort):
        try:
            self.serialConn.baudrate = 9600
            self.serialConn.port = serialPort
            self.serialConn.open()
        except:
            logging.debug("Com port already in use")

    def run(self):
        
        while self.stop_thread != True:

            try:
                item = self.rotationQueue.get_nowait()
                print(item)
                self.serialConn.write(item.encode())
            except:
                pass
            try:
                rotationData = self.serialConn.readline().decode().strip('\r\n').split("y")
            except:
                logging.debug("Cant recive rotation Arduino data ")
                rotationData = []
                pass
            
            if len(rotationData) == 5:
                try:
                    self.lock.acquire()
                    self.rotationSensor["topDriveMode"] = float(rotationData[0].split("x")[1])
                    self.rotationSensor["measuredRPM"] = float(rotationData[1])
                    self.rotationSensor["torqueMotor"] = float(rotationData[2])
                    self.rotationSensor["torqueSensor"] = float(rotationData[3])

                    self.lock.release()
                except:
                    try:
                        self.lock.release()
                    except:
                        pass
                    logging.debug("Cant place rotation Arduino data in dictionary")
                    print(rotationData)
                    pass
                    
                    
            
            

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
            "mode" :0,
            "pressure":0
        }
        self.circulationQueue = queue.Queue()
        self.serialConn = serial.Serial()
        self.stop_thread = False

    def setSerialPort(self,serialPort):
        try:
            self.serialConn.baudrate = 9600
            self.serialConn.port = serialPort
            self.serialConn.open()
        except:
            logging.debug("Com port already in use")

    def run(self):
        while self.stop_thread != True:
            try:
                item = self.circulationQueue.get_nowait()
                print(item)
                self.serialConn.write(item.encode())
            except:
                pass
            try:
                circulationData = self.serialConn.readline().decode().strip('\r\n').split("y")     

            except:
                logging.debug("Cant recive circulation arduino data ")
                circulationData = []
                pass
            
            if len(circulationData) == 2:
                
                try:
                    self.lock.acquire()
                    self.circulationSensor["mode"] = float(circulationData[0].split("x")[1])
                    self.circulationSensor["pressure"] = float(circulationData[1].split("z")[0])
                    
                    self.lock.release()
                except:
                    try:
                        self.lock.release()
                    except:
                        pass
                    logging.debug("Cant place rotation arduino data in dictonary")
                    print(circulationData)
                    pass

    def getCirculationSensorData(self):
        self.lock.acquire()
        cs = self.circulationSensor
        self.lock.release()
        return cs
    
