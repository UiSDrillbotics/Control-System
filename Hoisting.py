from enum import Enum
import HoistingData

import logging, sys
import math

logging.basicConfig(stream=sys.stderr, level= logging.DEBUG , datefmt='%Y-%m-%d %H:%M:%S')

class CommandType(Enum):
    STOP = 1
    CALIBRATE = 2
    MOVE = 3
    BRAKES = 4
    TELEMETRY = 6
    WOB = 7
    PID = 8
    RESETWOB = 10
    RESETSTEPPERS = 11
    HAMMER = 12
    AUTOMATE = 13

class Hoisting:
    def __init__(self, ArduinoHoistingData, ArduinoRotationData):
        self.hoistingData = HoistingData.HoistingData(
            brakeStatus = 0,
            x1 = 0,
            x2 = 0,
            x3 = 0,
            y1 = 0,
            y2 = 0,
            y3 = 0,
            z1 = 0,
            z2 = 0,
            z3 = 0,
            stepperArduinoPos1 = 0,
            stepperArduinoPos2 = 0,
            stepperArduinoPos3 = 0,
            hoistingMode = 0,
            heightSensor = 0,
            rop = 0,
            rop3m = 0

        )
        self.arduinoHoistingData = ArduinoHoistingData
        self.arduinoRotationData = ArduinoRotationData
        self.calibrated = False
        self.waitForBrakeStatus = False
        self.wobMode = None

        self.HardMinWOBLimit = 0
        self.HardMaxWOBLimit = 20

        self.KPropotinal = "0.00001"
        self.KIntegral = "0.0001"
        self.KDifferential = "0.00"

        self.WOBSetpoint = 0

        self.arduinoRailVoltage = 3.138
        
        self.hookLoad = 0

        self.measuredWOB = 0
    
    def move(self,distance,direction,speed,actuator):
        logging.info(actuator + " actuator(s) will " + direction + " " + distance + " mm with " + speed + " stepper delay")
        if direction == "Raise":
            direction = "1"
        if direction == "Lower":
            direction = "2"

        if actuator == "All":
            actuator = "4"
        command = str(CommandType.MOVE.value)
        output = command + ";" + distance + ";" + direction + ";" + speed + ";" + actuator + ";"+ "\n"
        self.arduinoHoistingData.hoistingQueue.put(output)



    def calibrate(self):
        command = str(CommandType.CALIBRATE.value)
        output = command + ";"+ "\n"
        self.arduinoHoistingData.hoistingQueue.put(output)
        logging.info("Calibrating hoisting system...")
        self.calibrated = True
    
    

    def brake(self,mode):
        command = str(CommandType.BRAKES.value)
        output = command + ";" + str(mode) + ";" + "\n"
        self.arduinoHoistingData.hoistingQueue.put(output)
        breakStatus = "Off"
        if mode ==2:
            breakStatus = "On"
        logging.info("Brakes " + breakStatus)
        self.waitForBrakeStatus = True

    def stop(self):
        command = str(CommandType.STOP.value)
        output = command + ";"+ "\n"
        self.arduinoHoistingData.hoistingQueue.put(output)
        logging.info("Hoisting stopping...")

    def wob(self, mode=-1):
        if mode ==-1:
            if self.wobMode == 1:
                mode = 0
            else:
                mode= 1
        
        command = str(CommandType.WOB.value)
        output = command + ";" + str(mode) + ";"+ "\n"
        self.arduinoHoistingData.hoistingQueue.put(output) 
        self.wobMode = mode
        logging.info("Wob mode set to " + str(mode))

    def telemetry(self):
        command = str(CommandType.TELEMETRY.value)
        output = command + ";"+ "\n"
        self.arduinoHoistingData.hoistingQueue.put(output)
        logging.info("Telementry turned on")

    def setWOB(self, WOB):
        if WOB < self.HardMinWOBLimit:
            WOB = self.HardMinWOBLimit
        if WOB > self.HardMaxWOBLimit:
            WOB = self.HardMaxWOBLimit
        self.setPID(WOB,self.KPropotinal, self.KIntegral, self.KDifferential)

    def setPID(self, WOB_Input, kp, ki, kd):
        if WOB_Input < self.HardMinWOBLimit:
            WOB_Input = self.HardMinWOBLimit
        if WOB_Input > self.HardMaxWOBLimit:
            WOB_Input = self.HardMaxWOBLimit
        if WOB_Input > 21:
            return
        
        self.arduinoHoistingData.WOBSetPoint = WOB_Input

        # WOB = (0.2288*WOB_Input*1000)+5034.3
        sumZ = self.arduinoHoistingData.getHoistingSensorData()["sumZ"]

        WOB = ((((sumZ+WOB_Input)*1000/3) + 7334.39633)/(4.3706)*3) - (((sumZ*1000/3) + 7334.39633)/(4.3706)*3)

        self.sendPID(int(WOB),kp,ki,kd)
    
    def sendPID(self, WOB, kp,ki,kd):
        self.KPropotinal = ki
        self.KIntegral = ki
        self.KDifferential = kd
        command = str(CommandType.PID.value)
        output = command + ";" + str(WOB) + ";" + kp + ";" + ki + ";" + kd + ";"+ "\n"
        self.arduinoHoistingData.hoistingQueue.put(output)
        logging.info("PID controller set with WOB: " + str(WOB) + " and parameters p: "+ kp + " i: "+ ki + " d: " + kd)

    def resetSteppers(self):
        pass
        #command = str(CommandType.RESETSTEPPERS.value)
        #output = command + ";"+ "\n"
        #self.arduinoHoistingData.hoistingQueue.put(output)
        #logging.info("Resetting steppers")
    
    def resetWOB(self):
        self.hookLoad = self.arduinoHoistingData.getHoistingSensorData()["sumZ"]
        self.arduinoHoistingData.WOBSetPoint = self.hookLoad
        command = str(CommandType.RESETWOB.value)
        output = command + ";"+ "\n"
        self.arduinoHoistingData.hoistingQueue.put(output)
        logging.info("Resetting WOB")
    
    def toggleHammerTime(self):
        command = str(CommandType.HAMMER.value)
        output = command + ";"+ "\n"
        self.arduinoHoistingData.hoistingQueue.put(output)
        logging.info("Toggling hammer time")

    def automate(self):
        command = str(CommandType.AUTOMATE.value)
        output = command + ";"+ "\n"
        self.arduinoHoistingData.hoistingQueue.put(output)
        logging.info("Initiazing atuomated drilling...")
    def getWOB(self):
        WOB = self.hookLoad - self.arduinoHoistingData.getHoistingSensorData()["sumZ"]
        self.measuredWOB = WOB
        return WOB

    def calcROP15s(self):
        data = self.arduinoHoistingData.getHoistingSensorData()["rop"]
        if len(data) > 0:
            return data[len(data)-1] - data[0]
        else:
            return 0
    
    def calcROP3m(self):
        data = self.arduinoHoistingData.getHoistingSensorData()["rop3m"]
        if len(data) > 0:
            return data[len(data)-1] - data[0]
        else:
            return 0

            # NB! Currently, the motor torque from the top drive is used as "bit torque". This must be calculated and updated into the next line:
    def calcMSE(self):
        if self.calcROP15s() == 0:
            MSE = 0
        else:
            try:         
                rotData = self.arduinoRotationData.getRotationSensorData()
                MSE = (((4*(self.getWOB()*9.81)/((math.pi*0.028575**2))) + (2*math.pi*rotData["torqueMotor"]*rotData["measuredRPM"])/(((math.pi/4)*(0.028575**2))*(self.calcROP15s()*0.06)))/1000000)
                # MSE = ((4*(self.getWOB())*9.8066500286389))/(math.pi*0.0286**2) + ((4*rotData["torqueMotor"]*rotData["measuredRPM"])/(math.pi*self.calcROP15s()*0.0286**2))
            except:
                MSE = 0        
        return MSE

    def calcUCS(self):
        UCS = 0.35*self.calcMSE()
        return UCS

    def calcDexponent(self):
        if self.calcROP15s() == 0:
            DEXP = 0
        else:
            try:         
                rotData = self.arduinoRotationData.getRotationSensorData()
                DEXP = math.log10(abs(self.calcROP15s())/(60*rotData["measuredRPM"])) / math.log10((12*abs(self.getWOB())/(1000*0.028)))
            except:
                DEXP = 0
        return DEXP

    def velocity(self):
        velocity = self.calcROP15s() * 3.6
        return velocity
