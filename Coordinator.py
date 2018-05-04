from enum import Enum
import logging,sys
import Hoisting
import Rotation
import Circulation
import ArduinoData
import threading
import time

logging.basicConfig(stream=sys.stderr, level= logging.DEBUG , datefmt='%Y-%m-%d %H:%M:%S')

class Problem(Enum):
    NormalAxialVibrations =1 
    DamagingAxialVibrations = 2
    LateralVibrations = 3
    StickSlip = 4
    Leak = 5
    Overpressure = 7
    Overpull = 8
    NoProblem = 9
    OffBottom = 10
 

class CoordinatorStates(Enum):

    JustStarted = 1     
    Calibrating = 2
    Drilling = 3
    Completed = 4
    Aborted = 5
    StartDrilling = 6
    TheEnd = 7


class Coordination(threading.Thread):

    def __init__(self, hoistingSystem, rotationSystem, circulationSystem, hoistingData, rotationData, circulationData):
        threading.Thread.__init__(self,daemon=True)
        self.runningThread = False
        self.hoistingSystem = hoistingSystem
        self.rotationSystem = rotationSystem
        self.circulationSystem = circulationSystem
        self.hoistingData = hoistingData
        self.rotationData = rotationData
        self.circulationData = circulationData
        #Init the system with no problems
        self.LastProblem = Problem.NoProblem
        #Init the system wit justStarted state
        self.coordinatorState = CoordinatorStates.JustStarted

        self.WOBincrease = 0.5
        self.RPMincrease  = 50

        self.MinRPMLimit = 50
        self.MaxRPMLimit = 1200
        self.MinWOBLimit = 0
        self.MaxWOBLimit = 10
        self.setpointCountdown = 1000

        self.calibrationStep = 0
        self.doneDrilling = False

        self.waitCountdown = 100
        self.position = 0
        self.fuckitCountDown = 50

        self.exceptPosition = 0
        #----------Variables for ROP Optimization---------#
        
        self.calculatingROP = False
        self.optimizeWOB = False
        self.optimizeRPM = True
        self.calculatingROPTimer = 0
        self.newROP = 0
        self.bestROP = 0
        self.instanceSinceBestROP = 0
        self.oldWOB = 0
        self.oldRPM = 0
    



    def run(self):
        #Thread for atuomated drilling
        while self.runningThread:
            self.setpointCountdown -=1

            #----------Just Started State---------#
            if CoordinatorStates.JustStarted == self.coordinatorState:
                self.calibrationStep = 0
                #The automate methode initiazies the procedure for calibratinon in the Arduino code
                self.hoistingSystem.automate()
                self.coordinatorState = CoordinatorStates.Calibrating
                
            #----------Calibrating State---------#
            if CoordinatorStates.Calibrating == self.coordinatorState:
                #Check sensor data for problems
                problem = self.lookForProblems()
                
                if Problem.NoProblem == problem:
                    self.makeCalibrationStep()
                    if self.areWeCalibratedYet():
                        self.calibrationStep = 5
                else:
                    self.mitigate(problem)

                if self.calibrationStep == 5:
                    self.coordinatorState = CoordinatorStates.StartDrilling
                    logging.debug("Coordinator " + str(self.coordinatorState))

            #----------Start Drilling State---------#
            if CoordinatorStates.StartDrilling == self.coordinatorState:
                
                self.rotationSystem.setRPM(500)
                self.circulationSystem.turnOnPump()
                self.hoistingSystem.setWOB(2.5)
                self.hoistingSystem.wob(1)
                self.coordinatorState = CoordinatorStates.Drilling
                logging.debug("coordinator" + str(self.coordinatorState))

            #----------Drilling State---------#
            if CoordinatorStates.Drilling == self.coordinatorState:

                if self.areWeFinishedYet():
                    self.coordinatorState = CoordinatorStates.Completed
                    logging.debug("coordinator" + str(self.coordinatorState))

                problem = self.lookForProblems()
                if Problem.NoProblem ==  problem:
                    #Continue with ROP optimization if no problem
                    self.optimizeROP()
                    
                else:
                    self.hoistingSystem.setWOB(2.5)
                    self.rotationSystem.setRPM(500)
                    logging.debug("coordinator" + str(self.coordinatorState))
                    logging.debug("problem" + str(problem))
                    self.mitigate(problem)
                
                anotherProblem = self.lookForProblems()
                if Problem.NoProblem == anotherProblem:
                    #self.hoistingSystem.wob(1)
                    pass
                
                else:
                    logging.debug("coordinator" + str(self.coordinatorState))
                    logging.debug("problem " + str(problem))
                    self.mitigate(problem)

            #----------Aborted State---------#
            if CoordinatorStates.Aborted == self.coordinatorState:
                self.turnOffSystem()
                self.coordinatorState = CoordinatorStates.JustStarted
                logging.debug("coordinator" + str(self.coordinatorState))
                
            #----------Completed State---------#
            if CoordinatorStates.Completed == self.coordinatorState:
                self.turnOffSystem()
                self.coordinatorState = CoordinatorStates.JustStarted
                logging.debug("coordinator" + str(self.coordinatorState))

    def turnOffSystem(self):
        self.hoistingSystem.wob(0)
        self.hoistingSystem.move("5","Raise","500","4")
        self.circulationSystem.turnOffPump()
        self.rotationSystem.setRPM(0)
        self.runningThread = False
    
    def makeCalibrationStep(self):
         
        if self.calibrationStep == 0:
            self.calibrationStep = 1
            logging.debug("CalibrationStep" + str(self.calibrationStep))

        if self.calibrationStep == 1:
            self.position = self.hoistingData.getHoistingSensorData()["stepperArduinoPos1"]
            self.hoistingSystem.move("2", "Raise", "500", "4") #up
            self.calibrationStep = 2
            logging.debug("CalibrationStep" + str(self.calibrationStep))

        if self.calibrationStep == 2:
            if self.hoistingData.getHoistingSensorData()["stepperArduinoPos1"] <= self.position - 1.99:
                self.hoistingSystem.resetWOB()
                self.hoistingSystem.setWOB(6)
                self.hoistingSystem.wob(1)
                self.calibrationStep = 3
                logging.debug("coordinator" + str(self.coordinatorState))
            
                    
        
        if self.calibrationStep == 3:
            if (self.hoistingSystem.getWOB() > (self.hoistingData.WOBSetPoint - 1)):
                self.hoistingSystem.wob(0)
                self.hoistingSystem.resetSteppers()
                self.position = self.hoistingData.getHoistingSensorData()["stepperArduinoPos1"]
                self.hoistingSystem.move("5", "Raise", "500", "4")
                self.fuckitCountDown = 50
                self.calibrationStep = 4
                logging.info("CalibrationStep" + str(self.calibrationStep))
              
        
        if self.calibrationStep == 4:
            if (self.hoistingData.getHoistingSensorData()["stepperArduinoPos1"] < (self.position - 4.99)):
                self.calibrationStep = 5
                logging.info("CalibrationStep" + str(self.calibrationStep))
                
            
        
    def areWeCalibratedYet(self):
        return (self.hoistingData.getHoistingSensorData()["hoistingMode"] == 8)
    
    def areWeFinishedYet(self):
        if self.hoistingData.getHoistingSensorData()["stepperArduinoPos1"] > 650 and self.hoistingData.taggedBottom == True and self.doneDrilling==False:
            return True
        else:
            return False
    
    def mitigateStickSlip(self):
        if self.setpointCountdown > 500:
            hoistData = self.hoistingData.getHoistingSensorData()
            distanceToRaise = 5.0
            self.hoistingSystem.wob(0)
            self.hoistingSystem.setWOB(self.hoistingData.WOBSetPoint - 0.5)
            self.exceptPosition = hoistData["stepperArduinoPos1"]
            self.hoistingSystem.move(str(distanceToRaise), "1.0", "400", "4.0")
            self.hoistingSystem.wob(1)  
        


    def mitigateOverpressure(self):

        hoistData = self.hoistingData.getHoistingSensorData()
        distanceToRaise = 5.0
        self.hoistingSystem.wob(0)
        self.exceptPosition = hoistData["stepperArduinoPos1"]
        self.hoistingSystem.move(str(distanceToRaise),"1.0",  "400", "4.0")


    def mitigate(self, problem):

        if Problem.DamagingAxialVibrations ==  problem:
            #rotationSystem.RPM = rotationSystem.RPM - RPMincrease * 0.5;
            #nMaxRPMLimit = MaxRPMLimit - RPMincrease * 0.45;
            if self.optimizeWOB:
                self.hoistingSystem.setRPM(self.oldWOB)
                self.optimizeWOB = False
                self.optimizeRPM = True
            else:
                self.rotationSystem.setRPM(self.oldRPM)
                self.optimizeRPM = False
                self.optimizeWOB = True

            pass
        if Problem.NormalAxialVibrations == problem:
            if self.optimizeWOB:
                self.hoistingSystem.setRPM(self.oldWOB)
                self.optimizeWOB = False
                self.optimizeRPM = True
            else:
                self.rotationSystem.setRPM(self.oldRPM)
                self.optimizeRPM = False
                self.optimizeWOB = True
            #rotationSystem.RPM -= RPMincrease / 2;
            pass
        if Problem.LateralVibrations == problem:
            if self.optimizeWOB:
                self.hoistingSystem.setRPM(self.oldWOB)
                self.optimizeWOB = False
                self.optimizeRPM = True
            else:
                self.rotationSystem.setRPM(self.oldRPM)
                self.optimizeRPM = False
                self.optimizeWOB = True
            pass
        if Problem.Leak ==  problem:
            self.coordinatorState = CoordinatorStates.Aborted
        if Problem.Overpressure ==  problem:
            self.mitigateOverpressure()
        if Problem.Overpull ==  problem:
            self.hoistingSystem.wob(0)
            self.hoistingSystem.move("0.1", "2", "400", "4")
            self.rotationSystem.setRPM(300)
        if Problem.StickSlip == problem:
            self.mitigateStickSlip()
        

    def lookForProblems(self):

        currentProblem = Problem.NoProblem
        hoistData = self.hoistingData.getHoistingSensorData()
        circData = self.circulationData.getCirculationSensorData()
        
        if self.rotationData.overTorqueCounter > 500:
            return Problem.StickSlip
        
        if hoistData["hoistingMode"] == 10.0:
            return Problem.DamagingAxialVibrations
        
        if hoistData["hoistingMode"] == 9.0:
            return Problem.NormalAxialVibrations

        if circData["mode"] == 4:
            return Problem.Leak
        
        if circData["mode"] == 3 or (self.LastProblem == Problem.Overpressure and hoistData["stepperArduinoPos1"] > self.exceptPosition):
            return Problem.Overpressure

        return currentProblem

    def optimizeROP(self):
        #Checks wether to manipulate wob or rpm
        if self.optimizeWOB:
            
            if not self.calculatingROP:
                newWOB = self.hoistingData.WOBSetPoint + 0.1
                if newWOB > self.MinWOBLimit and newWOB < self.MaxWOBLimit:
                    self.hoistingSystem.setWOB(newWOB)
                    self.hoistingSystem.wob(1)
                    self.calculatingROP = True
                    self.calculatingROPTimer = time.time()

            if self.calculatingROP and (time.time() - self.calculatingROPTimer >= 15):
                self.calculatingROP = False
                self.newROP = self.hoistingSystem.calcROP15s()
                if self.newROP >= self.bestROP:
                    self.instanceSinceBestROP = 0
                    self.bestROP = self.newROP
                    self.bestROPWOBSetPoint = self.hoistingData.WOBSetPoint
                if self.newROP < self.bestROP and self.instanceSinceBestROP >= 3:
                    self.optimizeWOB = False
                    self.optimizeRPM = True
                self.instanceSinceBestROP +=1
                self.oldWOB = self.hoistingData.WOBSetPoint
                
                
        if self.optimizeRPM:
            if not self.calculatingROP:
                newRPM = self.rotationSystem.setPointRPM + 10
                if newRPM> self.MinRPMLimit and newRPM < self.MaxRPMLimit:
                    self.rotationSystem.setRPM(newRPM)
                    self.calculatingROP = True
                    self.calculatingROPTimer = time.time()

            if self.calculatingROP and (time.time() - self.calculatingROPTimer >= 15):
                self.calculatingROP = False
                self.newROP = self.hoistingSystem.calcROP15s()
                if self.newROP >= self.bestROP:
                    self.instanceSinceBestROP = 0
                    self.bestROP = self.newROP
                    self.bestROPRPMSetpoint = self.rotationSystem.setPointRPM
                if self.newROP < self.bestROP and self.instanceSinceBestROP >= 3:
                    self.optimizeRPM = False
                    self.optimizeWOB = True
                self.instanceSinceBestROP +=1
                self.oldRPM = self.rotationSystem.setPointRPM

