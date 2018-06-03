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
    StickSlip = 13
    StuckPipe = 14
 

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

        self.MinRPMLimit = 400 # THESE VALUES ARE WHERE WE DETERMINE CONSTRAINTS FOR UNI-VARIATE SEARCH ALGORITHMS
        self.MaxRPMLimit = 1250
        self.MinWOBLimit = 2
        self.MaxWOBLimit = 15
        self.setpointCountdown = 20

        self.wobList = [i for i in np.arange(self.MinWOBLimit, self.MaxWOBLimit, self.WOBincrease)]
        self.rpmList = [i for i in np.arange(self.MinRPMLimit, self.MaxRPMLimit, self.RPMincrease)]
        #Rpm index 2 = 500 rpm
        self.rpmIndex = 2
        #WOB index 5 = 5 wob
        self.wobIndex = 6
        self.calibrationStep = 0
        self.doneDrilling = False
        self.MSEList = []
        self.ROPList = []

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
        self.stick_slip_counter = 0
        self.ropChange = True

    def run(self):
        #Thread for atuomated drilling
        while self.runningThread:
            self.setpointCountdown -=1
            #Mse list used for new formation detection
            if self.fillList:
                self.MSEList.append(self.hoistingSystem.calcMSE())
                self.ROPList.append(self.hoistingData.getHoistingSensorData()["rop"])
                if self.setpointCountdown <= 0:
                    self.numberCountDown +=1
                    if self.numberCountDown >= 11: # THIS IS WHERE WE DECIDE HOW MANY SECONDS TO LOOK FOR MIN-MAX DIFFERENCE (validate MSE) - NB! THIS IS FOR EACH OF TWO WINDOWS
                        self.numberCountDown = 0
                        self.fillList = False
                        self.stick_slip_counter = 0
                        self.ropChange = True
            else:
                self.MSEList.pop(0)
                self.MSEList.append(self.hoistingSystem.calcMSE())
                self.ROPList.pop(0)
                self.ROPList.append(self.hoistingData.getHoistingSensorData["rop"])

            if self.setpointCountdown <= 0:
                self.problemFlag = False
                self.newFormation = False
                self.rotationData.overTorqueCounter = 0
                self.rotationData.overBrakeCounter = 0
                self.oldTorque = self.rotationData.getRotationSensorData()["torqueMotor"]
                #aprox 1 sec with 20 Hz update rate
                self.setpointCountdown = 20
                self.ongoingProblem = Problem.NoProblem
            time.sleep(0.05)
            #----------Just Started State---------#
            if CoordinatorStates.JustStarted == self.coordinatorState:
                self.calibrationStep = 1
                #The automate methode initiazies the procedure for calibratinon in the Arduino code
                #self.hoistingSystem.automate()
                self.coordinatorState = CoordinatorStates.Calibrating
                
            #----------Calibrating State---------#
            if CoordinatorStates.Calibrating == self.coordinatorState:
                #Check sensor data for problems
                problem = self.lookForProblems()
                
                if Problem.NoProblem == problem:
                    self.makeCalibrationStep()
     
                        
                else:
                    self.mitigate(problem)

                if self.calibrationStep == 5:
                    self.coordinatorState = CoordinatorStates.StartDrilling
                    logging.debug("Coordinator " + str(self.coordinatorState))

            #----------Start Drilling State---------#
            if CoordinatorStates.StartDrilling == self.coordinatorState:
                
                self.hoistingSystem.setWOB(self.wobList[self.wobIndex])
                self.oldRPM = 500
                self.oldWOB = 5
                time.sleep(1)
                self.circulationSystem.turnOnPump()
                time.sleep(1)
                self.rotationSystem.setRPM(self.rpmList[self.rpmIndex])
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
                if Problem.NoProblem !=  problem:
                    if not self.problemFlag:
                        logging.debug("coordinator" + str(self.coordinatorState))
                        logging.debug("problem" + str(problem))
                        self.mitigate(problem)
                        self.problemFlag = True

                self.optimizeROP()

                if (self.hoistingData.getHoistingSensorData()["TVD"])/10 >= 2: #only checks for formations after 2 cm in the well
                    self.newFormation = self.lookForNewFormation()


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
        if self.hoistingData.getHoistingSensorData()["TVD"] > 210 and self.doneDrilling==False:
            logging.info("Done Drilling!")
            return True
        else:
            return False


    def mitigateStickSlip(self):
        self.hoistingSystem.setWOB(0)
        self.hoistingSystem.move("5", "Raise", "100", "4")
        time.sleep(3)
        for _ in range(0,3):
            newWOB = self.manipulateWOB("decrease")

        
        self.hoistingSystem.setWOB(newWOB)


        self.hoistingSystem.wob(1)
        self.stick_slip_counter += 1


    def mitigateStuckPipe(self):
        print("STUCCC PIPE")
        self.hoistingSystem.setWOB(0)
        self.hoistingSystem.move("15", "Raise", "100", "4")
        time.sleep(8)
        #Set wob and RPM to initial values
        self.rpmIndex = 2
        self.wobIndex = 6

        self.hoistingSystem.setWOB(self.wobList[self.wobIndex])
        self.rotationSystem.setRPM(self.rpmList[self.rpmIndex])

        self.hoistingSystem.wob(1)
        self.stick_slip_counter = 0


    def mitigate(self, problem):

        if Problem.DamagingAxialVibrations ==  problem:
            #rotationSystem.RPM = rotationSystem.RPM - RPMincrease * 0.5;
            #nMaxRPMLimit = MaxRPMLimit - RPMincrease * 0.45;
            if self.optimizeWOB:
                self.hoistingSystem.setWOB(self.manipulateWOB("decrease"))
                self.optimizeWOB = False
                self.optimizeRPM = True
                self.numberOfInc = 0
            else:
                self.rotationSystem.setRPM(self.manipulateRPM("decrease"))
                self.optimizeRPM = False
                self.optimizeWOB = True
                self.numberOfInc = 0

        if Problem.NormalAxialVibrations == problem:
            if self.optimizeWOB:
                self.hoistingSystem.setWOB(self.manipulateWOB("decrease"))
                self.optimizeWOB = False
                self.optimizeRPM = True
                self.numberOfInc = 0
            else:
                self.rotationSystem.setRPM(self.manipulateRPM("decrease"))
                self.optimizeRPM = False
                self.optimizeWOB = True
                self.numberOfInc = 0
        if Problem.LateralVibrations == problem:
            if self.optimizeWOB:
                self.hoistingSystem.setWOB(self.manipulateWOB("decrease"))
                self.optimizeWOB = False
                self.optimizeRPM = True
                self.numberOfInc = 0
            else:
                self.rotationSystem.setRPM(self.manipulateRPM("decrease"))
                self.optimizeRPM = False
                self.optimizeWOB = True
                self.numberOfInc = 0
            pass
        if Problem.Leak ==  problem:
            self.coordinatorState = CoordinatorStates.Aborted
            
        if Problem.Overpressure ==  problem:
            if self.optimizeWOB:
                self.hoistingSystem.setWOB(self.manipulateWOB("decrease"))
                self.optimizeWOB = False
                self.optimizeRPM = True
                self.numberOfInc = 0
            else:
                self.rotationSystem.setRPM(self.manipulateRPM("decrease"))
                self.optimizeRPM = False
                self.optimizeWOB = True
                self.numberOfInc = 0
        if Problem.Overpull ==  problem:
            self.hoistingSystem.wob(0)
            self.hoistingSystem.move("0.1", "2", "400", "4")
            self.rotationSystem.setRPM(300)
        if Problem.TorsionalVibrations == problem:
            if self.optimizeWOB:
                self.hoistingSystem.setWOB(self.manipulateWOB("decrease"))
                self.optimizeWOB = False
                self.optimizeRPM = True
                self.numberOfInc = 0
            else:
                self.rotationSystem.setRPM(self.manipulateRPM("decrease"))
                self.optimizeRPM = False
                self.optimizeWOB = True
                self.numberOfInc = 0

        if Problem.MotorOverLoad == problem:
            self.coordinatorState = CoordinatorStates.Aborted
        

        if Problem.StuckPipe == problem:
            self.mitigateStuckPipe()

        if Problem.StickSlip == problem:
            self.mitigateStickSlip()


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
        
        if rotData["torqueMotor"] >=5.5:
            self.rotationData.overBrakeCounter += 1

        if self.rotationData.overBrakeCounter >=3:
            self.ongoingProblem = Problem.StickSlip
            return Problem.StickSlip

        if circData["mode"] == 4:
            self.ongoingProblem = Problem.Leak
            return Problem.Leak
        
        if circData["mode"] == 3:
            self.ongoingProblem = Problem.Overpressure
            return Problem.Overpressure


        if self.stick_slip_counter >=3:
            self.ongoingProblem = Problem.StuckPipe
            return Problem.StuckPipe


        return currentProblem
    
    def lookForNewFormation(self):
        try:
            rop1 = self.ROPList[0]
            rop2 = self.ROPList[len(self.ROPList)-1]
            if self.ropChange:
                change = (rop1-rop2)/rop1
                if change > 0.8:
                    self.rpmIndex = 2
                    self.wobIndex = 6
                    self.hoistingSystem.setWOB(self.wobList[self.wobIndex])
                    self.rotationSystem.setRPM(self.rpmList[self.rpmIndex])

                    self.hoistingSystem.wob(1)
                    self.ropChange = False
                    return True

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
            if change > 0.6: # THIS IS WHERE WE DECIDE THE PERTCENTAGE ERROR IN MSE OVER A TIME WINDOW THAT TRIGGER NEW FORMATION
                return True
            else:
                return False
        except:
            return False
            
    
    def manipulateWOB(self,command):
        newWOB = 0
        if command == "increase":
            if self.wobIndex+1 < len(self.wobList):
                self.wobIndex += 1
                newWOB = self.wobList[self.wobIndex]
                
            else:
                newWOB = self.wobList[self.wobIndex]
        elif command=="decrease":
            if self.wobIndex -1 >= 0:
                self.wobIndex -= 1 
                newWOB = self.wobList[self.wobIndex]
                
            else:
                newWOB = self.wobList[self.wobIndex]
        else:
            logging.info("Bad WOB command")
            newWOB = self.wobList[self.wobIndex]
        return newWOB


    def manipulateRPM(self,command):
        newRPM = 0
        if command == "increase":
            if self.rpmIndex+1 < len(self.rpmList):
                self.rpmIndex += 1
                newRPM = self.rpmList[self.rpmIndex]
                
            else:
                newRPM = self.rpmList[self.rpmIndex]
        elif command=="decrease":
            if self.rpmIndex -1 >= 0:
                self.rpmIndex -= 1 
                newRPM = self.rpmList[self.rpmIndex]
                
            else:
                newRPM = self.rpmList[self.rpmIndex]
        else:
            logging.info("Bad RPM command")
            newRPM = self.rpmList[self.rpmIndex]

        return newRPM

    def optimizeROP(self):
        #Checks wether to manipulate wob or rpm
        if self.optimizeWOB:
            
            if not self.calculatingROP:
                newWOB = self.manipulateWOB("increase")
                if newWOB >= self.MinWOBLimit and newWOB <= self.MaxWOBLimit and self.numberOfInc < 5:
                    self.hoistingSystem.setWOB(newWOB)
                    self.hoistingSystem.wob(1)
                    self.calculatingROP = True
                    self.calculatingROPTimer = time.time()
                    self.numberOfInc+= 1
                else:
                    self.optimizeRPM = True
                    self.optimizeWOB = False
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
                
                
        if self.optimizeRPM:
            if not self.calculatingROP:
                
                newRPM = self.manipulateRPM("increase")
                if newRPM >= self.MinRPMLimit and newRPM <= self.MaxRPMLimit and self.numberOfInc < 5:
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


