from enum import Enum
import logging,sys
import Hoisting
import Rotation

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
    Calibrating = 2 #going to the top, hitting the push buttons
    Drilling = 3
    Completed = 4
    Aborted = 5
    StartDrilling = 6
    TheEnd = 7


class Coordination():

    def __init__(self, hoistingSystem, rotationSystem, circulationSystem):

        self.hoistingSystem = hoistingSystem
        self.rotationSystem = rotationSystem
        self.circulationSystem = circulationSystem
        #Init the system with no problems
        self.LastProblem = Problem.NoProblem
        #Init the system wit justStarted state
        self.coordinatorState = CoordinatorStates.JustStarted

        self.sweepingActive = None
        self.waitForWOBControl = None
        self.stickSlip = None
        self.dExponent = None
        self.ccs = None
        self.ucs = None
        self.mse = None
        self.WOBincrease = 0.5
        self.RPMincrease  = 50
        self.waitingToMoveUp = None
        self.doneDrilling = None
        self.stickSlipThread = None
        self.MinRPMLimit = None
        self.MaxRPMLimit = None
        self.MinWOBLimit = None
        self.MaxWOBLimit = None
        self.nMinRPMLimit = None
        self.nMaxRPMLimit = None
        self.nMinWOBLimit = None
        self.nMaxWOBLimit = None

        self.setpointCountdown = 1000

        self.calibrationStep = 0

        self.waitCountdown = 100
        self.position = None
        self.fuckitCountDown = 50

        self.exceptPosition = None
    
    def manageStickSlip(self):
        self.stickSlip = True
        self.hoistingSystem.wob(0)
        #timer here
        self.hoistingSystem.move(distance=0.5, direction = "1.0", speed = 400, actuator = "4.0")

    def NormalProcedure(self):
        self.setpointCountdown -=1

        if isinstance(CoordinatorStates.JustStarted,self.coordinatorState):
            self.calibrationStep = 0

        if isinstance(CoordinatorStates.Calibrating,self.coordinatorState):
            if self.calibrationStep == 5:
                self.coordinatorState = CoordinatorStates.StartDrilling
                logging.debug("Coordinator " + self.coordinatorState)

            problem = self.lookForProblems()
            if isinstance(Problem.NoProblem,problem):
                self.makeCalibrationStep()
            else:
                self.nMaxWOBLimit = self.MaxWOBLimit
                self.nMinWOBLimit = self.MinWOBLimit
                self.nMaxRPMLimit = self.MaxRPMLimit
                self.nMinRPMLimit = self.MinRPMLimit

                self.mitigate(problem)

                self.MaxWOBLimit = self.nMaxWOBLimit
                self.MinWOBLimit = self.nMinWOBLimit
                self.MaxRPMLimit = self.nMaxRPMLimit
                self.MinRPMLimit = self.nMinRPMLimit

        if isinstance(CoordinatorStates.StartDrilling,self.coordinatorState):
            self.rotationSystem.RPM = 500
            self.circulationSystem.turnOnPump()
            self.hoistingSystem.setWOB(2.5)
            self.hoistingSystem.wob(1)
            self.coordinatorState = CoordinatorStates.Drilling
            logging.debug("coordinator" + self.coordinatorState)
        
        if isinstance(CoordinatorStates.Drilling, self.coordinatorState):

            if self.areWeFinishedYet():
                self.coordinatorState = CoordinatorStates.Completed
                logging.debug("coordinator" + self.coordinatorState)

            problem = self.lookForProblems()
            if isinstance(Problem.NoProblem, problem):
                if (self.setpointCountdown< 1):
                    self.rotationSystem.RPM = self.rotationSystem.RPM +1 
                    self.hoistingSystem.setWOB(self.hoistingSystem.WOBSetpoint + 1)
                    self.setpointCountdown = 1000

                self.hoistingSystem.wob(1)
            else:
                self.hoistingSystem.setWOB(2.5)
                self.rotationSystem.RPM = 500
                logging.debug("coordinator" + self.coordinatorState)
                logging.debug("problem" + problem)

                self.nMaxWOBLimit = self.MaxWOBLimit
                self.nMinWOBLimit = self.MinWOBLimit
                self.nMaxRPMLimit = self.MaxRPMLimit
                self.nMinRPMLimit = self.MinRPMLimit

                self.mitigate(problem)

                self.MaxWOBLimit = self.nMaxWOBLimit
                self.MinWOBLimit = self.nMinWOBLimit
                self.MaxRPMLimit = self.nMaxRPMLimit
                self.MinRPMLimit = self.nMinRPMLimit
            
            anotherProblem = self.lookForProblems()
            if isinstance(Problem.NoProblem,anotherProblem):
                self.hoistingSystem.wob(1)
            
            else:
                logging.debug("coordinator " + self.coordinatorState)
                logging.debug("problem " + problem)
                self.nMaxWOBLimit = self.MaxWOBLimit
                self.nMinWOBLimit = self.MinWOBLimit
                self.nMaxRPMLimit = self.MaxRPMLimit
                self.nMinRPMLimit = self.MinRPMLimit

                self.mitigate(problem)

                self.MaxWOBLimit = self.nMaxWOBLimit
                self.MinWOBLimit = self.nMinWOBLimit
                self.MaxRPMLimit = self.nMaxRPMLimit
                self.MinRPMLimit = self.nMinRPMLimit

        if isinstance(CoordinatorStates.Aborted,self.coordinatorState):
            self.turnOffSystem()
            self.coordinatorState = CoordinatorStates.JustStarted
            logging.debug("coordinator " + self.coordinatorState)
        if isinstance(CoordinatorStates.Completed,self.coordinatorState):
            self.turnOffSystem()
            self.coordinatorState = CoordinatorStates.JustStarted
            logging.debug("coordinator " + self.coordinatorState)

    def turnOffSystem(self):
        self.hoistingSystem.wob(0)
        self.hoistingSystem.move(5,"1",500,"4")
        self.circulationSystem.turnOffPump()
        self.rotationSystem.RPM = 0
    
    def makeCalibrationStep(self):
         
        if self.calibrationStep == 0:
            self.calibrationStep = 1
            logging.debug("CalibrationStep" + self.calibrationStep)

        if self.calibrationStep == 1:
            self.position = self.hoistingSystem.getData().stepperArduinoPosition1
            self.hoistingSystem.move(2, "1", 500, "4") #up
            self.calibrationStep = 2
            logging.debug("CalibrationStep" + self.calibrationStep)

        if self.calibrationStep == 2:
            if self.hoistingSystem.getData().stepperArduinoPosition1 <= self.position - 1.99:
                if self.waitCountdown <= 0:
                    self.hoistingSystem.resetWOB()
                    self.hoistingSystem.setWOB(6)
                    self.hoistingSystem.wob(1)
                    self.calibrationStep = 3
                    logging.debug("CalibrationStep" + self.calibrationStep)
                
                self.waitCountdown -=1
        
        if self.calibrationStep == 3:
            if (self.fuckitCountDown == 0) or (self.hoistingSystem.WOB > (self.hoistingSystem.WOBSetpoint - 1)):
                self.hoistingSystem.wob(0)
                self.hoistingSystem.resetSteppers()
                self.position = self.hoistingSystem.getData().stepperArduinoPosition1
                self.hoistingSystem.move(5, "1", 500, "4")
                self.fuckitCountDown = 50
                self.calibrationStep = 4
                logging.debug("CalibrationStep" + self.calibrationStep)
            self.fuckitCountDown -= 1
        
        if self.calibrationStep == 4:
            if (self.fuckitCountDown == 0) or (self.hoistingSystem.getData().stepperArduinoPosition1 < (self.position - 4.99)):
                self.calibrationStep = 5
                logging.debug("CalibrationStep" + self.calibrationStep)
            self.fuckitCountDown -= 1
        
    def areWeCalibratedYet(self):
        return (self.hoistingSystem.getData().hoistingMode == 8)
    
    def areWeFinishedYet(self):
        if self.hoistingSystem.getData().stepperArduinoPosition1 > 650 and self.hoistingSystem.taggedBottom == True and self.doneDrilling==False:
            return True
        else:
            return False
    
    def mitigateStickSlip(self):

        self.nMaxWOBLimit = self.MaxWOBLimit
        self.nMinWOBLimit = self.MinWOBLimit
        self.nMaxRPMLimit = self.MaxRPMLimit
        self.nMinRPMLimit = self.MinRPMLimit

        if self.setpointCountdown > 500:
            hoistData = self.hoistingSystem.getData()
            distanceToRaise = 5.0
            self.hoistingSystem.wob(0)
            self.hoistingSystem.setWOB(self.hoistingSystem.WOBSetpoint - 0.5)
            self.exceptPosition = hoistData.stepperArduinoPosition1
            self.hoistingSystem.move(distance = distanceToRaise, direction = "1.0", speed = 400, actuator = "4.0")
            self.hoistingSystem.wob(1)  
    
    def mitigateOverpressure(self):
        self.nMaxWOBLimit = self.MaxWOBLimit
        self.nMinWOBLimit = self.MinWOBLimit
        self.nMaxRPMLimit = self.MaxRPMLimit
        self.nMinRPMLimit = self.MinRPMLimit
        hoistData = self.hoistingSystem.getData()
        distanceToRaise = 5.0
        self.hoistingSystem.wob(0)
        self.exceptPosition = hoistData.stepperArduinoPosition1
        self.hoistingSystem.move(distance = distanceToRaise, direction = "1.0", speed = 400, actuator = "4.0")


    def mitigate(self, problem):
        self.nMaxWOBLimit = self.MaxWOBLimit
        self.nMinWOBLimit = self.MinWOBLimit
        self.nMaxRPMLimit = self.MaxRPMLimit
        self.nMinRPMLimit = self.MinRPMLimit

        if isinstance(Problem.DamagingAxialVibrations, problem):
            #rotationSystem.RPM = rotationSystem.RPM - RPMincrease * 0.5;
            #nMaxRPMLimit = MaxRPMLimit - RPMincrease * 0.45;
            pass
        if isinstance(Problem.NormalAxialVibrations,problem):
            #rotationSystem.RPM -= RPMincrease / 2;
            pass
        if isinstance(Problem.LateralVibrations, problem):
            pass
        if isinstance(Problem.Leak, problem):
            self.coordinatorState = CoordinatorStates.Aborted
        if isinstance(Problem.Overpressure, problem):
            self.mitigateOverpressure()
        if isinstance(Problem.Overpull, problem):
            self.hoistingSystem.wob(0)
            self.hoistingSystem.move(0.1, "2", 400, "4")
            self.rotationSystem.RPM = 300
        if isinstance(Problem.StickSlip, problem):
            self.mitigateStickSlip()
        
        self.MaxWOBLimit = self.nMaxWOBLimit
        self.MinWOBLimit = self.nMinWOBLimit
        self.MaxRPMLimit = self.nMaxRPMLimit
        self.MinRPMLimit = self.nMinRPMLimit
    
    def lookForProblems(self):

        currentProblem = Problem.NoProblem
        hoistData = self.hoistingSystem.getData()

        if self.rotationSystem.overTorqueCounter > 500:
            return Problem.StickSlip
        
        if hoistData.hoistingMode == 10.0:
            return Problem.DamagingAxialVibrations
        
        if hoistData.hoistingMode == 9.0:
            return Problem.NormalAxialVibrations

        if self.circulationSystem.Mode == 4:
            return Problem.Leak
        
        if self.circulationSystem.Mode == 3 or (self.LastProblem == Problem.Overpressure and hoistData.stepperArduinoPosition1 > self.exceptPosition):
            return Problem.Overpressure

        return currentProblem

    def automate(self):
        self.NormalProcedure()

