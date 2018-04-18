import ArduinoData
import logging,sys


class Circulation:
    def __init__(self,arduinoCirculationData):
        self.pumpOn = False
        self.pump2Mode = 0
        self.mode = 0
        self.oldMode = 0
        self.ArduinoRotationData = arduinoCirculationData

    
    def turnOffPump(self):
        self.pumpOn = False
        self.ArduinoRotationData.circulationQueue.put("0")
        logging.info("Turning off pump")

    def turnOnPump(self):
        self.pumpOn = True
        self.ArduinoRotationData.circulationQueue.put("1")
        logging.info("Turning on pump")
        


