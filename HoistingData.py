arduinoRailVoltage = 3.138

calibratedX1 = 1.0257 * arduinoRailVoltage
calibratedY1 = 1.0099 * arduinoRailVoltage
calibratedZ1 = 1.1513 * arduinoRailVoltage
calibratedX2 = 1.0402 * arduinoRailVoltage
calibratedY2 = 1.0452 * arduinoRailVoltage
calibratedZ2 = 1.1311 * arduinoRailVoltage
calibratedX3 = 1.0170 * arduinoRailVoltage
calibratedY3 = 0.9983 * arduinoRailVoltage
calibratedZ3 = 1.1453 * arduinoRailVoltage
class HoistingData:


    def __init__(self, brakeStatus,x1,x2,x3,y1,y2,y3,z1,z2,z3,
        stepperArduinoPos1,stepperArduinoPos2,stepperArduinoPos3,
        hoistingMode,heightSensor,rop,rop3m):

        self.brakeStatus = brakeStatus
        self.x1 = (((x1 * (3.3 / 4096)) - (calibratedX1 / 2.0)) * (200 / calibratedX1)) * 0.101971621 # add (*1000) to get readings in gram.
        self.y1 = (((y1 * (3.3 / 4096)) - (calibratedY1 / 2.0)) * (200 / calibratedY1)) * 0.101971621 # add (*1000) to get readings in gram.
        self.z1 = (((z1 * (3.3 / 4096)) - (calibratedZ1 / 2.0)) * (200 / calibratedZ1)) * 0.101971621 # add (*1000) to get readings in gram.
        self.x2 = (((x2 * (3.3 / 4096)) - (calibratedX2 / 2.0)) * (200 / calibratedX2)) * 0.101971621 # add (*1000) to get readings in gram.
        self.y2 = (((y2 * (3.3 / 4096)) - (calibratedY2 / 2.0)) * (200 / calibratedY2)) * 0.101971621 # add (*1000) to get readings in gram.
        self.z2 = (((z2 * (3.3 / 4096)) - (calibratedZ2 / 2.0)) * (200 / calibratedZ2)) * 0.101971621 # add (*1000) to get readings in gram.
        self.x3 = (((x3 * (3.3 / 4096)) - (calibratedX3 / 2.0)) * (200 / calibratedX3)) * 0.101971621 # add (*1000) to get readings in gram.
        self.y3 = (((y3 * (3.3 / 4096)) - (calibratedY3 / 2.0)) * (200 / calibratedY3)) * 0.101971621 # add (*1000) to get readings in gram.
        self.z3 = (((z3 * (3.3 / 4096)) - (calibratedZ3 / 2.0)) * (200 / calibratedZ3)) * 0.101971621

        #doing self to the integers in arduino now
        #z1 = (0.9525 * z1) + 0.0906 #self is old calibration equation
        #z3 = (0.9806 * z3) + 0.0234 #self is calibration equation
        y1 = 0
        x3 = 0
        sumX = self.x1 + self.x2
        sumY = self.y2 + self.y3
        sumZ = self.z1 + self.z2 + self.z3
        self.stepperArduinoPosition1 = stepperArduinoPos1
        self.stepperArduinoPosition2 = stepperArduinoPos2
        self.stepperArduinoPosition3 = stepperArduinoPos3
        self.hoistingMode = hoistingMode
        self.heightSensor = heightSensor
        self.rop = rop
        self.rop3m = rop3m