import ArduinoData
import logging,sys

logging.basicConfig(stream=sys.stderr, level= logging.DEBUG, )

class Rotation():

    def __init__(self, arduinoRotationData):
        self.measuredRPM = 0
        self.TDBitTorque = 0
        self.torqueMotor = 0
        self.topDriveMode = 0
        self.torqueSensor = 0
        self.TDDPTorque = 0

        self.HardMinRPMLimit = 50
        self.HardMaxRPMLimit  = 750

        self.setPointRPM = 0
        self.ArduinoRotationData = arduinoRotationData
    
    def setRPM(self,rpm):
        if rpm == 0:
            self.setPointRPM = rpm
            self.ArduinoRotationData.rotationQueue.put(self.setPointRPM)
            logging.info("Stopped rotation")
        else:
            if rpm > self.HardMaxRPMLimit:
                self.setPointRPM = self.HardMaxRPMLimit
            elif rpm < self.HardMinRPMLimit:
                self.setPointRPM = self.HardMinRPMLimit
            else:
                self.setPointRPM = rpm
            self.ArduinoRotationData.rotationQueue.put(self.setPointRPM)
            logging.info("RPM set to " + str(self.setPointRPM))


        
    
    def calculateTDBitTorque(self):

        if self.measuredRPM != 0:
            self.TDBitTorque = self.torqueMotor - ((0.0015 * self.measuredRPM) + 0.7175)
        else:
            self.TDBitTorque = 0
    
    def calculateTDDPTorque(self):

        if self.measuredRPM != 0:
            self.TDDPTorque = self.torqueMotor - ((-0.0000008 * (self.measuredRPM * self.measuredRPM)) + (0.0009 * self.measuredRPM) + 0.9539)
        else:
            self.TDDPTorque = 0
    
