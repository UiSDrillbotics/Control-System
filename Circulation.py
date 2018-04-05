import ArduinoData
import logging,sys


class Circulation:
    def __init__(self,arduinoCirculationData):
        self.pump1Mode = 0
        self.pump2Mode = 0
        self.mode = 0
        self.oldMode = 0
        self.ArduinoRotationData = arduinoCirculationData

    
    def turnOffPump(self):
        self.pump1Mode = 0
        self.pump2Mode = 0
        self.ArduinoRotationData.circulationQueue.put("0")
        logging.info("Turning off pump")

    def turnOnPump(self):
        if self.pump1Mode == 0 and self.pump2Mode==0:
            self.pump1Mode = 1
        else:
            print("The pumps are already on")
        self.ArduinoRotationData.circulationQueue.put("1")
        logging.info("Turning on pump")
        


    def swichPump(self):
        if self.pump1Mode == 0 and self.pump2Mode == 1:
            self.pump1Mode = 1
            self.pump2Mode = 0
        elif self.pump1Mode == 1 and self.pump2Mode==0:
            self.pump1Mode = 0
            self.pump2Mode = 1
        else:
            print("Turn on pumps before switch")
