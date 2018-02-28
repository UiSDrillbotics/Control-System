from enum import Enum
import HoistingData

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
    def __init__(self):
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
    
    def move(self,distance,direction,speed,actuator):
        command = CommandType.MOVE
        output = command + ";" + distance + ";" + direction + ";" + speed + ";" + actuator + ";"


    def calibrate(self):
        command = CommandType.CALIBRATE
        output = command + ";"
    
    

    def brake(self,mode):
        command = CommandType.BRAKES.name
        return command + ";" + str(mode)

    def stop(self):
        command = CommandType.STOP
        output = command + ":"

    def wob(self):
        command = CommandType.WOB
        output = command + ";"

    def telemetry(self):
        command = CommandType.TELEMETRY
        output = command + ";"

    def setWOB(self, WOB):
        return

