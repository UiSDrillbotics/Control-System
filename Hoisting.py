from enum import Enum
import HoistingData
import logging, sys

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
    def __init__(self, ArduinoHoistingData):
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
        self.calibrated = False
        self.waitForBrakeStatus = False
        self.wobMode = None

        self.HardMinWOBLimit = 2
        self.HardMaxWOBLimit = 6

        self.KPropotinal = "0.0001"
        self.KIntegral = "0.0001"
        self.KDifferential = "0.00"

        self.WOBSetpoint = None

        self.arduinoRailVoltage = 3.138
        
        self.hookLoad = 0

        self.measuredWOB = 0
    
    def move(self,distance,direction,speed,actuator):
        logging.info(actuator + " actuator(s) will " + direction + " " + distance + " mm with " + speed + " stepper delay")
        if direction == "Raise":
            direction = "1"
        else:
            direction = "2"

        if actuator == "All":
            actuator = "4"
        command = str(CommandType.MOVE.value)
        output = command + ";" + distance + ";" + direction + ";" + speed + ";" + actuator + ";"
        self.arduinoHoistingData.hoistingQueue.put(output)



    def calibrate(self):
        command = str(CommandType.CALIBRATE.value)
        output = command + ";"
        self.arduinoHoistingData.hoistingQueue.put(output)
        logging.info("Calibrating hoisting system...")
        self.calibrated = True
    
    

    def brake(self,mode):
        command = str(CommandType.BRAKES.value)
        output = command + ";" + str(mode) + ";" + "\r"
        self.arduinoHoistingData.hoistingQueue.put(output)
        breakStatus = "Off"
        if mode ==2:
            breakStatus = "On"
        logging.info("Brakes " + breakStatus)
        self.waitForBrakeStatus = True

    def stop(self):
        command = str(CommandType.STOP.value)
        output = command + ";"
        self.arduinoHoistingData.hoistingQueue.put(output)
        logging.info("Hoisting stopping...")

    def wob(self, mode=-1):
        if mode ==-1:
            if self.wobMode == 1:
                mode = 0
            else:
                mode= 1
        
        command = str(CommandType.WOB.value)
        output = command + ";" + str(mode) + ";"
        self.arduinoHoistingData.hoistingQueue.put(output) 
        self.wobMode = mode
        logging.info("Wob mode set to " + str(mode))

    def telemetry(self):
        command = str(CommandType.TELEMETRY.value)
        output = command + ";"
        self.arduinoHoistingData.hoistingQueue.put(output)
        logging.info("Telementry turned on")

    def setWOB(self, WOB):
        if WOB < self.HardMinWOBLimit:
            WOB = self.HardMinWOBLimit
        if WOB > self.HardMaxWOBLimit:
            WOB = self.HardMaxWOBLimit
        self.setPID(WOB,self.KPropotinal, self.KIntegral, self.KPropotinal)

    def setPID(self, WOB_Input, kp, ki, kd):
        if WOB_Input < self.HardMinWOBLimit:
            WOB_Input = self.HardMinWOBLimit
        if WOB_Input > self.HardMaxWOBLimit:
            WOB_Input = self.HardMaxWOBLimit
        if WOB_Input > 15:
            return
        
        self.WOBSetpoint = WOB_Input
        WOB = (WOB_Input/0.101971621)*(self.arduinoRailVoltage/200) * (4096/3.3)
       
        self.sendPID(int(WOB),kp,ki,kd)
    
    def sendPID(self, WOB, kp,ki,kd):
        self.KPropotinal = ki
        self.KIntegral = ki
        self.KDifferential = kd
        command = str(CommandType.PID.value)
        output = command + ";" + str(WOB) + ";" + kp + ";" + ki + ";" + kd + ";"
        self.arduinoHoistingData.hoistingQueue.put(output)
        logging.info("PID controller set with WOB: " + str(WOB) + " and parameters p: "+ kp + " i: "+ ki + " d: " + kd)

    def resetSteppers(self):
        command = str(CommandType.RESETSTEPPERS.value)
        output = command + ";"
        self.arduinoHoistingData.hoistingQueue.put(output)
        logging.info("Resetting steppers")
    
    def resetWOB(self):
        self.hookLoad = self.arduinoHoistingData.getHoistingSensorData()["sumZ"]
        command = str(CommandType.RESETWOB.value)
        output = command + ";"
        self.arduinoHoistingData.hoistingQueue.put(output)
        logging.info("Resetting WOB")
    
    def toggleHammerTime(self):
        command = str(CommandType.HAMMER.value)
        output = command + ";"
        self.arduinoHoistingData.hoistingQueue.put(output)
        logging.info("Toggling hammer time")

    def getWOB(self):
        WOB = self.hookLoad - self.arduinoHoistingData.getHoistingSensorData()["sumZ"]
        self.measuredWOB = WOB
        return WOB
