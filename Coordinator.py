from enum import Enum
import logging,sys
import Hoisting
import Rotation
import Circulation
import ArduinoData
import threading
import time
import numpy as np

logging.basicConfig(stream=sys.stderr, level= logging.DEBUG , datefmt='%Y-%m-%d %H:%M:%S')

class Problem(Enum):
    NormalAxialVibrations =1 
    DamagingAxialVibrations = 2
    LateralVibrations = 3
    TorsionalVibrations = 4
    Leak = 5
    Overpressure = 7
    Overpull = 8
    NoProblem = 9
    OffBottom = 10
    MotorOverLoad = 11
    TwistOff = 12
 

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
        self.ongoingProblem = Problem.NoProblem
        self.WOBincrease = 0.5
        self.RPMincrease  = 50

        self.MinRPMLimit = 50
        self.MaxRPMLimit = 1200
        self.MinWOBLimit = 0
        self.MaxWOBLimit = 10
        self.setpointCountdown = 20

        self.wobList = [i for i in np.arange(self.MinWOBLimit, self.MaxWOBLimit, self.WOBincrease)]
        self.rpmList = [i for i in np.arange(self.MinRPMLimit, self.MaxRPMLimit, self.RPMincrease)]
        self.calibrationStep = 0
        self.doneDrilling = False
        self.MSEList = []

        self.waitCountdown = 100
        self.position = 0
        self.problemFlag = False
        self.safeTagBottom = 0
        self.exceptPosition = 0
        self.oldTorque = 0
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
        self.numberOfInc = 0
        
    

        self.fillList = True
        self.newFormation = False
        self.numberCountDown = 0

    def run(self):
        #Thread for atuomated drilling
        while self.runningThread:
            self.setpointCountdown -=1
            #Mse list used for new formation detection
            if self.fillList:
                self.MSEList.append(self.hoistingSystem.calcMSE())
                if self.setpointCountdown <= 0:
                    self.numberCountDown +=1
                    if self.numberCountDown >= 6:
                        self.numberCountDown = 0
                        self.fillList = False
            else:
                self.MSEList.pop(0)
                self.MSEList.append(self.hoistingSystem.calcMSE())

            if self.setpointCountdown <= 0:
                self.newFormation = False
                self.rotationData.overTorqueCounter = 0
                self.oldTorque = self.rotationData.getRotationSensorData()["torqueMotor"]
                #aprox 1 sec with 20 Hz update rate
                self.setpointCountdown = 20
                self.ongoingProblem = Problem.NoProblem
            time.sleep(0.05)
            #----------Just Started State---------#
            if CoordinatorStates.JustStarted == self.coordinatorState:
                self.calibrationStep = 0
                #The automate methode initiazies the procedure for calibratinon in the Arduino code
                #self.hoistingSystem.automate()
                self.coordinatorState = CoordinatorStates.Calibrating
                
            #----------Calibrating State---------#
            if CoordinatorStates.Calibrating == self.coordinatorState:
                #Check sensor data for problems
                problem = self.lookForProblems()
                
                if Problem.NoProblem == problem:
                    self.makeCalibrationStep()
                    pass
     
                        
                else:
                    self.mitigate(problem)

                if self.calibrationStep == 5:
                    self.coordinatorState = CoordinatorStates.StartDrilling
                    logging.debug("Coordinator " + str(self.coordinatorState))

            #----------Start Drilling State---------#
            if CoordinatorStates.StartDrilling == self.coordinatorState:
                
                self.hoistingSystem.setWOB(5)
                self.oldRPM = 450
                self.oldWOB = 2.0
                time.sleep(1)
                self.circulationSystem.turnOnPump()
                time.sleep(1)
                self.rotationSystem.setRPM(500)
                time.sleep(1)
                self.hoistingSystem.wob(1)
                self.coordinatorState = CoordinatorStates.Drilling
                logging.debug("coordinator" + str(self.coordinatorState))

            #----------Drilling State---------#
            if CoordinatorStates.Drilling == self.coordinatorState:

                if self.areWeFinishedYet():
                    self.coordinatorState = CoordinatorStates.Completed
                    logging.debug("coordinator" + str(self.coordinatorState))

                problem = self.lookForProblems()
                self.optimizeROP()
                if Problem.NoProblem ==  problem:
                    #Continue with ROP optimization if no problem
                    
                    pass
                else:
                    if not self.problemFlag:
                        #self.hoistingSystem.setWOB(2.5)
                        #self.rotationSystem.setRPM(500)
                        logging.debug("coordinator" + str(self.coordinatorState))
                        logging.debug("problem" + str(problem))
                        #self.mitigate(problem)
                        self.problemFlag = True

                self.newFormation = self.lookForNewFormation()
                anotherProblem = self.lookForProblems()
                if Problem.NoProblem == anotherProblem:
                    #self.hoistingSystem.wob(1)
                    self.problemFlag = False
                
                else:
                    logging.debug("coordinator" + str(self.coordinatorState))
                    logging.debug("problem " + str(problem))
                    #self.mitigate(problem)

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
            
            print(self.hoistingData.getHoistingSensorData()["stepperArduinoPos1"])
            print(self.position)
            if self.hoistingData.getHoistingSensorData()["stepperArduinoPos1"] <= self.position - 1:
                self.hoistingSystem.resetWOB()
                time.sleep(1)
                self.hoistingSystem.setWOB(6)
                time.sleep(1)
                self.hoistingSystem.wob(1)
                
                self.calibrationStep = 3
                logging.debug("coordinator" + str(self.coordinatorState))
            
                    
        
        if self.calibrationStep == 3:
            if (self.hoistingSystem.getWOB() > (self.hoistingData.WOBSetPoint)):
                self.safeTagBottom += 1
                if self.safeTagBottom  >= 4:

                    self.hoistingSystem.wob(0)
                    time.sleep(1)
                    self.hoistingData.resetTVD()
                    
                    time.sleep(1)
                    self.position = self.hoistingData.getHoistingSensorData()["stepperArduinoPos1"]
                    self.hoistingData.oldTVD = self.hoistingData.getHoistingSensorData()["stepperArduinoPos1"]
                    self.hoistingSystem.move("5", "Raise", "300", "4")
                    time.sleep(1)
                    self.fuckitCountDown = 50
                    self.calibrationStep = 4
                    logging.info("CalibrationStep" + str(self.calibrationStep))
                    self.safeTagBottom = 0
                
        
        if self.calibrationStep == 4:
            if (self.hoistingData.getHoistingSensorData()["stepperArduinoPos1"] < (self.position - 4.98)):
                self.calibrationStep = 5
                logging.info("CalibrationStep" + str(self.calibrationStep))
                
            
        
    def areWeCalibratedYet(self):
        return (self.hoistingData.getHoistingSensorData()["hoistingMode"] == 8)
    
    def areWeFinishedYet(self):
        if self.hoistingData.getHoistingSensorData()["stepperArduinoPos1"] > 650 and self.doneDrilling==False:
            return True
        else:
            return False
    
    def mitigateTorsionalVibrations(self):

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
                self.hoistingSystem.setWOB(self.oldWOB)
                self.optimizeWOB = False
                self.optimizeRPM = True
            else:
                self.rotationSystem.setRPM(self.oldRPM)
                self.optimizeRPM = False
                self.optimizeWOB = True

            pass
        if Problem.NormalAxialVibrations == problem:
            if self.optimizeWOB:
                self.hoistingSystem.setWOB(self.oldWOB)
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
                self.hoistingSystem.setWOB(self.oldWOB)
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
        if Problem.TorsionalVibrations == problem:
            self.mitigateTorsionalVibrations()

        if Problem.MotorOverLoadCounter == problem:
            self.coordinatorState = CoordinatorStates.Aborted

    def lookForProblems(self):

        currentProblem = Problem.NoProblem
        hoistData = self.hoistingData.getHoistingSensorData()
        circData = self.circulationData.getCirculationSensorData()
        rotData = self.rotationData.getRotationSensorData()
        if self.oldTorque != 0:
            if ((rotData["torqueMotor"]/self.oldTorque) > 1.5) and circData["mode"] == 4:
                self.ongoingProblem = Problem.TwistOff
                return Problem.TwistOff

        if rotData["torqueMotor"] >= 7:
            self.rotationData.overTorqueCounter += 1

        if rotData["torqueMotor"] >= 8.59:
            self.rotationData.motorOverLoadCounter += 1

        if self.rotationData.overTorqueCounter > 2:
            self.ongoingProblem = Problem.TorsionalVibrations
            return Problem.TorsionalVibrations
        
        if self.rotationData.motorOverLoadCounter > 20 and rotData["measuredRPM"] <= 50:
            self.ongoingProblem = Problem.MotorOverLoad
            return Problem.MotorOverLoad

        
        if hoistData["hoistingMode"] == 10.0:
            self.ongoingProblem = Problem.DamagingAxialVibrations
            return Problem.DamagingAxialVibrations
        
        if hoistData["hoistingMode"] == 9.0:
            self.ongoingProblem = Problem.NormalAxialVibrations
            return Problem.NormalAxialVibrations

        if circData["mode"] == 4:
            self.ongoingProblem = Problem.Leak
            return Problem.Leak
        
        if circData["mode"] == 3 or (self.LastProblem == Problem.Overpressure and hoistData["stepperArduinoPos1"] > self.exceptPosition):
            self.ongoingProblem = Problem.Overpressure
            return Problem.Overpressure

        return currentProblem
    
    def lookForNewFormation(self):
        halfIndex = int(len(self.MSEList)/2)
        maxMseWindow1 = max(self.MSEList[:halfIndex])
        minMSeWindow1 = min(self.MSEList[:halfIndex])

        maxMseWindow2 = max(self.MSEList[halfIndex:])   
        minMSeWindow2 = min(self.MSEList[halfIndex:])

        diffWindow1 = maxMseWindow1 - minMSeWindow1
        diffWindow2 = maxMseWindow2 - minMSeWindow2
        if diffWindow1 == 0 or diffWindow2 == 0:
            return False
        change = 0
        if diffWindow1 > diffWindow2:
            change = (diffWindow1-diffWindow2)/diffWindow1
        else:
            change = (diffWindow2-diffWindow1)/diffWindow2
        if change > 0.4:
            print("New Formation")
            return True
        else:
            return False
        

    def optimizeROP(self):
        #Checks wether to manipulate wob or rpm
        if self.optimizeWOB:
            
            if not self.calculatingROP:
                newWOB = self.hoistingData.WOBSetPoint + self.WOBincrease
                if newWOB > self.MinWOBLimit and newWOB < self.MaxWOBLimit and self.numberOfInc < 5:
                    self.hoistingSystem.setWOB(newWOB)
                    self.hoistingSystem.wob(1)
                    self.calculatingROP = True
                    self.calculatingROPTimer = time.time()
                    self.numberOfInc+= 1
                else:
                    self.optimizeRPM = False
                    self.optimizeWOB = True
                    self.numberOfInc = 0

            if self.calculatingROP and (time.time() - self.calculatingROPTimer >= 5):
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
                
                newRPM = self.rotationSystem.setPointRPM + self.RPMincrease
                if newRPM> self.MinRPMLimit and newRPM < self.MaxRPMLimit and self.numberOfInc < 5:
                    self.rotationSystem.setRPM(newRPM)
                    self.calculatingROP = True
                    self.calculatingROPTimer = time.time()
                    self.numberOfInc +=1
                else:
                    self.optimizeRPM = False
                    self.optimizeWOB = True
                    self.numberOfInc = 0

            if self.calculatingROP and (time.time() - self.calculatingROPTimer >= 5):
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

