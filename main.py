import Circulation
import Hoisting
import threading
import ArduinoData
import time
import sys
import pyqtdesign
import VisualizationGUI
import controls
import pyqtgraph as pg
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QThread,pyqtSignal,QTime
from PyQt5.QtWidgets import QApplication, QMainWindow
import Drillbotics2018
import glob
import math

import Hoisting
import Circulation
import Rotation
import Coordinator

import database
from Classifier import Classify

oldTVD = 0
oldWOBset = 0
oldRPMset = 0
#Lock for each arduino data storage
hoistigLock = threading.Lock()
circulationLock = threading.Lock()
rotationLock = threading.Lock()

#Init each thread for reading arduino data
t1 = ArduinoData.HoistingData(hoistigLock)
t2 = ArduinoData.CirculationData(circulationLock)
t3 = ArduinoData.RotationData(rotationLock)

#Init each system responsible for actions related to each Arduino
#Each system is initialized with the ArduinoData thread that they use
hoistingSystem = Hoisting.Hoisting(t1,t3)
circulationSystem = Circulation.Circulation(t2)
rotationSystem = Rotation.Rotation(t3)
coordinationSystem = Coordinator.Coordination(hoistingSystem,rotationSystem,circulationSystem,t1,t3,t2)

#Initiazises the database 
db = database.Database(t1,t3,t2,hoistingSystem,circulationSystem,rotationSystem) 
db.initDb()

#Init the classifier
classifier = Classify(t1)
##Gets data and triggers the plot
class GetData(QThread):
    dataChanged = pyqtSignal(bool,int,float,float,float,str,float,float,float,float,float,float,float,float,float,float,float,
        float, float, float, float,float,float,float,float,float,float,float,float,float,float)
    def __init__(self, parent=None):
        QThread.__init__(self, parent)
        self.t = QTime()
    def __del__(self):  # part of the standard format of a QThread
        self.wait()
    def run(self):  
        self.t.start()
        while True:
            #Get the dictonarys from the arduinoData module
            hSensorData = t1.getHoistingSensorData()
            cSensorData = t2.getCirculationSensorData()
            rSensorData = t3.getRotationSensorData()
            #Init each variable with the correspondable dictonary value
            RPM = rSensorData["measuredRPM"]
            if RPM <=0:
                RPM = 0
            torqueMotor = rSensorData["torqueMotor"]
            torqueSensor = rSensorData["torqueSensor"]
            pressure = cSensorData["pressure"]

            hoistingMode = hSensorData["hoistingMode"]
            coordinatorProblem  = coordinationSystem.ongoingProblem.value

            topDriveMode = rSensorData["topDriveMode"]
            circulationMode = cSensorData["mode"]
            Z1 = hSensorData["z1"]
            Z2 = hSensorData["z2"]
            Z3 = hSensorData["z3"]
            rock = classifier.predicedLabel

            rock = str(rock)
            newFormation = coordinationSystem.newFormation
            #print(rock)
            sumZ = hSensorData["sumZ"]
            ROP_15s = (hoistingSystem.calcROP15s())/15

            ROP_3m = (hoistingSystem.calcROP3m())/180
            # 3. degree polynomial Q = (-72.831*pressure**3) + (499.29*pressure**2) - (1124.5*pressure) + 842.08
            # 2. degree polynomial Q = (-19.109*pressure**2) + (95.095*pressure) - 106.56
            
            if circulationSystem.pumpOn == False:
                Q = 0

            else:
                Q = (3.7417*pressure) + 0.7122
            
            MSE = hoistingSystem.calcMSE() 
            UCS = hoistingSystem.calcUCS()
            TVD = hSensorData["TVD"]/10
            torqueBit = torqueMotor # must be updated here, and updated in hoistingSystem
            WOB = hoistingSystem.getWOB()
            dExponenet = hoistingSystem.calcDexponent()
            Height = hSensorData["heightSensor"]

            act1 = hSensorData["stepperArduinoPos1"]
            act2 = hSensorData["stepperArduinoPos2"]
            act3 = hSensorData["stepperArduinoPos3"]
            
            vibration = 0
            velocity = hoistingSystem.velocity()

            timeNow = float(self.t.elapsed())/(1000*60)
            if TVD == 0:
                timeNow = 0
                #self.t = QTime()
                self.t.start()

            wobSetpoint = float(t1.WOBSetPoint)
            rpmSetpoint =  float(rotationSystem.setPointRPM)
            #Sleeps to not overload the system
            time.sleep(0.1)
          
            #Sends the new data to the chart and labels in the HMI
            self.dataChanged.emit(newFormation,coordinatorProblem,circulationMode,topDriveMode,hoistingMode,rock,wobSetpoint,rpmSetpoint,velocity,TVD,act1,act2,act3,Height,Q,ROP_15s,ROP_3m,Z1,Z2,Z3,
                sumZ,WOB,pressure,torqueMotor,RPM,vibration,MSE,UCS,torqueBit,dExponenet,timeNow) 
            #Triggers and updates the plot and labels


class ControlUI(QWidget,Drillbotics2018.Ui_C):
    def __init__(self, parent=None):
        QWidget.__init__(self,parent)
        #Sets up the control interface(buttons, brakes, etc.)
        self.setupUi(self)
        #Initializes and starts the thread responible for getting data to the HMI
        self.dataThread = GetData(self)
        self.dataThread.start()
        #Sets up the connection between the data change thread and the HMI data update
        self.dataThread.dataChanged.connect(self.updateLabels)
        #Functions for buttons in the HMI
        self.pushButton_Advanced_UI.clicked.connect(self.showAdvancedUI)
        self.pushButton_OpenPorts.clicked.connect(self.getComPorts)
        self.pushButton_StartRotation.clicked.connect(self.setRPM)
        self.pushButton_Stop_Rotation.clicked.connect(self.stopRotation)
        self.pushButton_MoveActuators.clicked.connect(self.moveHoisting)
        #self.pushButton_Calibrate.clicked.connect(self.calibrate)
        self.pushButton_Open_Brake.clicked.connect(self.openBrake)
        self.pushButton_Close_Brake.clicked.connect(self.closeBreak)
        self.pushButton_Stop_Hoisting.clicked.connect(self.stopHoisting)
        self.pushButton_WOB_ctrl_on.clicked.connect(self.wobControlOn)
        self.pushButton_WOB_Ctrl_off.clicked.connect(self.wobControlOff)
        self.pushButton_Reset_WOB_2.clicked.connect(self.resetWOB)
        self.pushButton_Telementary.clicked.connect(self.telementary)
        self.pushButton_SavePID.clicked.connect(self.setPIDController)
        self.pushButton_Reset_Stepper_Pos.clicked.connect(self.resetSteppers)
        self.pushButton_StartCirculation.clicked.connect(self.turnOnPump)
        self.pushButton_Stop_Pump.clicked.connect(self.turnOffPump)
        self.pushButton_Start_Drilling.clicked.connect(self.startDrilling)
        self.pushButton_Stop_Drilling.clicked.connect(self.stopDrilling)
        self.pushButton_Reset_Hook_Load.clicked.connect(self.resetHookLoad)
        self.pushButton
        #Gets the serial ports available in the operating system
        #Temporary feature: Gets prots for MAC OS is testing occures on a MAC
        if sys.platform.startswith('win'):
            import serial.tools.list_ports_windows
            ports = serial.tools.list_ports_windows.comports()
        elif sys.platform.startswith('linux'):
            import serial.tools.list_ports_linux
            
            ports = serial.tools.list_ports_linux.comports()
            #print(temp_list)
            
        else:
            import serial.tools.list_ports_osx
            ports = serial.tools.list_ports_osx.comports()
        #Stores the connected ports for the HMI to show
        connectedPorts = []
        for element in ports:
            connectedPorts.append(str(element.device))
        #Populates the comboboxes with connected ports
        self.comboBox_Rotation.addItems(['/dev/ttyACM0']) # should be (connectedPorts) by default
        self.comboBox_Circulation.addItems(['/dev/ttyACM1']) # should be (connectedPorts) by default
        self.comboBox_Hoisting.addItems(['/dev/ttyACM2']) # should be (connectedPorts) by default

        #Populates Accuator combobox
        self.comboBox_Actuator.addItems(["All","1","2","3"])

        #Populates the Driection combobox
        self.comboBox_Direction.addItems(["Raise", "Lower"])

    #Method for showing the advanced UI
    def showAdvancedUI(self):
        self.window = QtWidgets.QWidget()
        #Init the advanced UI window the the same datathread as the controls window
        self.ui = GUI(self.dataThread)
        self.ui.show()
        self.visGUI = VisGUI(self.dataThread)
        self.visGUI.show()

    def getComPorts(self):
        
        hoistingPort = self.comboBox_Hoisting.currentText()
        circulationPort = self.comboBox_Circulation.currentText()
        rotationPort = self.comboBox_Rotation.currentText()
        #Get ports from the combobox and initialize connections with the ardionos in the arduinoData module
        t1.setSerialPort(hoistingPort)
        t2.setSerialPort(circulationPort)
        t3.setSerialPort(rotationPort)
        #Start each thread for getting and sending Arduino data 
        t1.start()
        t2.start()
        t3.start()
        #Starts the database thread
        db.start()
        classifier.start()
        self.label_Brake_Open.setStyleSheet("background-color: yellow")
        self.label_Brake_Closed.setStyleSheet("background-color: white")
        self.pushButton_OpenPorts.setDisabled(True)
    #Method for reading the selected RPM and send it to the Rotation module
    def setRPM(self):
        rpm = self.doubleSpinBox_RPM.value()
        rotationSystem.setRPM(int(rpm))

    def stopRotation(self):

        rotationSystem.setRPM(0)
        
    def moveHoisting(self):
        actuator = self.comboBox_Actuator.currentText()
        distance = str(float(self.doubleSpinBox_Distance.value()))
        stepperDelay = str(int(self.spinBox_Stepper_Delay.value()))
        direction = self.comboBox_Direction.currentText()
        if actuator != "All":
            #Check if the user if sure he/she want to move only one actuator
            mb = QtGui.QMessageBox
            question = mb.question(self,'', "Are you sure you want to move only one actuator?", mb.Yes | mb.No)
            if question == mb.Yes:
                hoistingSystem.move(distance,direction,stepperDelay,actuator)
        else:
            hoistingSystem.move(distance,direction,stepperDelay,actuator)

    #def calibrate(self):
     #   hoistingSystem.calibrate()

    def openBrake(self):
        hoistingSystem.brake(2)

    def closeBreak(self):
        hoistingSystem.brake(1)

    def stopHoisting(self):
        hoistingSystem.stop()

    def wobControlOn(self):
        hoistingSystem.wob(1)

    def wobControlOff(self):
        hoistingSystem.wob(0)
    
    def resetWOB(self):
        hoistingSystem.resetWOB()

    def telementary(self):
        hoistingSystem.telemetry()
    
    def setPIDController(self):
        wob = self.doubleSpinBox_WOB.value()
        kp = self.doubleSpinBox_Kp.value()
        ki = self.doubleSpinBox_Ki.value()
        kd = self.doubleSpinBox_Kd.value()

        hoistingSystem.setPID(wob,str(kp),str(ki),str(kd))
        
    def resetSteppers(self):
        hoistingSystem.resetSteppers()
        t1.resetTVD()
        global oldTVD
        oldTVD  = -1.1

    def turnOnPump(self):
        
        circulationSystem.turnOnPump()
    
    def turnOffPump(self):
        circulationSystem.turnOffPump()

    def resetHookLoad(self):
        t1.resetHookload()

    def startDrilling(self):
        if not coordinationSystem.runningThread:
            coordinationSystem.runningThread = True
            coordinationSystem.start()
            
    def stopDrilling(self):
        coordinationSystem.turnOffSystem()
        coordinationSystem.join()
        mb = QtGui.QMessageBox
        mb.information(self,' ',"Automated drilling is terimated, restart the system for a new drilling process",  mb.Ok | mb.Cancel)
 
    
    def updateLabels(self,newFormation,coordinatorProblem,circulationMode,topDriveMode,hoistingMode,rock,wobSetpoint,rpmSetpoint,velocity,TVD,act1,act2,act3,Height,Q,ROP_15s,ROP_3m,Z1,Z2,Z3,sumZ,WOB,Pressure,Torque,RPM,Vibration,MSE,UCS,torqueBit,dExponenet,timeNow):
        WOB = float("{0:.2f}".format(WOB))
        ROP_15s = float("{0:.2f}".format(ROP_15s))
        ROP_3m = float("{0:.2f}".format(ROP_3m))
        MSE = float("{0:.2f}".format(MSE))
        UCS = float("{0:.2f}".format(UCS))
        Q = float("{0:.2f}".format(Q))
        velocity = float("{0:.2f}".format(velocity))
        dExponenet = float("{0:.2f}".format(dExponenet))

        self.label_WOB.setText(str(WOB))
        self.label_Pressure.setText(str(Pressure))
        self.label_Topdrive_Torque.setText(str(Torque))
        self.label_RPM.setText(str(RPM))
        self.label_Flow_Rate_Q.setText(str(Q))
        self.label_LoadCell_Z1.setText(str(int(Z1)))
        self.label_LoadCell_Z2.setText(str(int(Z2)))
        self.label_LoadCell_Z3.setText(str(int(Z3)))
        self.label_Hookload_SumZ.setText(str(int(sumZ)))
        self.label_ROP15.setText(str(ROP_15s))
        self.label_ROP3.setText(str(ROP_3m))
        self.label_Height_Sensor.setText(str(Height))
        self.label_UCS.setText(str(UCS))
        self.label_MSE.setText(str(MSE))
        self.label_d_exponent.setText(str(dExponenet))
        self.label_Act1StepCount.setText(str(act1))
        self.label_Act2StepCount.setText(str(act2))
        self.label_Act3StepCount.setText(str(act3))
        self.Depthtracker.display(TVD)
        self.Depthtracker_2.display(timeNow)
        self.label_Velocity.setText(str(velocity))
        self.label_StepCounter.setText(str(act1))
        self.textBrowser_CurrentFormation.setPlainText(rock)

        if TVD >= 0 and TVD < 10:
            if not self.radioButton.isChecked():
                self.radioButton.toggle()
        elif TVD >= 10 and TVD < 20:
            if not self.radioButton_2.isChecked():
                self.radioButton_2.toggle()
        elif TVD >= 20 and TVD < 30:
            if not self.radioButton_3.isChecked():
                self.radioButton_3.toggle()
        elif TVD >= 30 and TVD < 40:
            if not self.radioButton_5.isChecked():
                self.radioButton_5.toggle()
        elif TVD >= 40 and TVD < 50:
            if not self.radioButton_6.isChecked():
                self.radioButton_6.toggle()
        elif TVD >= 50 and TVD < 60:
            if not self.radioButton_7.isChecked():
                self.radioButton_7.toggle()
        elif TVD >= 60:
            if not self.radioButton_9.isChecked():
                self.radioButton_9.toggle()
        else:
            pass

        if hoistingMode == 0:
            self.label_Start.setStyleSheet("background-color: yellow")
        else:
            self.label_Start.setStyleSheet("background-color: white")
        if hoistingMode == 1:
            self.label_Calibrating.setStyleSheet("background-color: yellow")
        else:
            self.label_Calibrating.setStyleSheet("background-color: white")
        if hoistingMode == 2:
            self.label_Moving.setStyleSheet("background-color: yellow")
        else:
            self.label_Moving.setStyleSheet("background-color: white")
        if hoistingMode == 3:
            self.label_Hoisting_Stopped.setStyleSheet("background-color: yellow")
        else:
            self.label_Hoisting_Stopped.setStyleSheet("background-color: white")
        if hoistingMode == 4:
            self.label_WOB_Ctrl.setStyleSheet("background-color: yellow")
        else:
            self.label_WOB_Ctrl.setStyleSheet("background-color: white")
        if hoistingMode == 10:
            self.label_Axial_Vibration.setStyleSheet("background-color: red")
        else:
            self.label_Axial_Vibration.setStyleSheet("background-color: white")
         
        if circulationMode == 0:
            self.label_Pump_Off.setStyleSheet("background-color: yellow")
        else:
            self.label_Pump_Off.setStyleSheet("background-color: white")

        if circulationMode == 1:
            self.label_Starting.setStyleSheet("background-color: yellow")
            self.label_Pump_On.setStyleSheet("background-color: yellow")
        else:
            self.label_Starting.setStyleSheet("background-color: white")
            self.label_Pump_On.setStyleSheet("background-color: white")
        
        if circulationMode == 2:
            self.label_Monitor.setStyleSheet("background-color: yellow")
            self.label_Pump_On.setStyleSheet("background-color: yellow")
        else:
            self.label_Monitor.setStyleSheet("background-color: white")
            self.label_Pump_On.setStyleSheet("background-color: white")

        if circulationMode == 3:
            self.label_Overpressure.setStyleSheet("background-color: red")

        else:
            self.label_Overpressure.setStyleSheet("background-color: white")
        
        if circulationMode == 4:
            self.label_Leak.setStyleSheet("background-color: red")
            self.label_LEAK.setStyleSheet("background-color: yellow")
        else:
            self.label_Leak.setStyleSheet("background-color: white")
            self.label_LEAK.setStyleSheet("background-color: white")

 
        if topDriveMode == 0 or rpmSetpoint == 0:
            self.label_No_Rotation.setStyleSheet("background-color: yellow")
            self.label_Rotating_Bit.setStyleSheet("background-color: white")
        else:
            self.label_No_Rotation.setStyleSheet("background-color: white")
            self.label_Rotating_Bit.setStyleSheet("background-color: yellow")
        
        if coordinatorProblem == 4:
            self.label_Stick_Slip.setStyleSheet("background-color: red")
        else:
            self.label_Stick_Slip.setStyleSheet("background-color: white")
        if coordinatorProblem == 11:
            self.label_Over_Torque.setStyleSheet("background-color: red")
        else:
            self.label_Over_Torque.setStyleSheet("background-color: white")
        if coordinatorProblem == 12:
            self.label_Twist_Off.setStyleSheet("background-color: red")
        else:
            self.label_Twist_Off.setStyleSheet("background-color: white")
        
        if newFormation:
            self.label_New_Formation.setStyleSheet("background-color: green")
        else:
            self.label_New_Formation.setStyleSheet("background-color: white")

        

class VisGUI(QMainWindow,VisualizationGUI.Ui_MainWindow):
    def __init__(self,dataThread,parent=None):
        QMainWindow.__init__(self,parent)
        self.setupUi(self)
        self.dataThread = dataThread
        self.dataThread.dataChanged.connect(self.updateLabels)

        self.initTVD = True        
        self.oldTVD  = 0

        #List of values that will be displayed in the graph
        self.RPM = []
        self.torque = []
        self.bit_torque = []
        self.pressure = []
        self.TVD = []
        self.WOB = []
        self.MSE = []
        self.ROP = []
        self.UCS = []
        self.wobSet = []
        self.rpmSet = []
        self.ropSet = []
        self.mseSet = []
        
        #Init the RPM plot
        layoutRPM = QHBoxLayout()
        
        self.RPMPlot = pg.PlotWidget()
        self.RPMPlot.setYRange(0, 45)
        self.RPMPlot.setXRange(0,1500)
        layoutRPM.addWidget(self.RPMPlot)
        self.graphicsView_adv_RPM.setLayout(layoutRPM) # Places the plot in the graphisView from the desinger

        self.p1 = self.RPMPlot.plotItem

        self.p1.showGrid(x = True, y = True, alpha = 0.7)   
        self.RPMCurve = self.p1.plot()
        self.RPMCurve.setPen(pg.mkPen(color="#fff000", width=2))
        self.RPMCurve.getViewBox().invertY(True)
        
        self.p1.getAxis('right').setLabel('RPM', color='#0000ff')

        #Init the Torque plot
        layoutTorque = QHBoxLayout()
        
        self.torquePlot = pg.PlotWidget()
        self.torquePlot.setYRange(0, 45)
        self.torquePlot.setXRange(0,4)
        layoutTorque.addWidget(self.torquePlot)
        self.graphicsView_adv_torque.setLayout(layoutTorque)

        
        self.p2 = self.torquePlot.plotItem
  
        self.p2.showGrid(x = True, y = True, alpha = 0.7)   
        self.torqueCurve = self.p2.plot()
        self.torqueCurve.setPen(pg.mkPen(color="#fff000", width=2))
        self.torqueCurve.getViewBox().invertY(True)
        self.p2.getAxis('right').setLabel('RPM', color='#0000ff')

        #Init the ROP plot
        layoutROP = QHBoxLayout()
        
        self.ROPPlot = pg.PlotWidget()
        self.ROPPlot.setYRange(0, 45)
        self.ROPPlot.setXRange(0,80)
        layoutROP.addWidget(self.ROPPlot)
        self.graphicsView_adv_ROP.setLayout(layoutROP)

        
        self.p3 = self.ROPPlot.plotItem

        self.p3.showGrid(x = True, y = True, alpha = 0.7)   
        self.ROPCurve = self.p3.plot()
        self.ROPCurve.setPen(pg.mkPen(color="#fff000", width=2))
        self.ROPCurve.getViewBox().invertY(True)
        self.p3.getAxis('right').setLabel('ROP', color='#0000ff')

        #Init the WOB plot
        layoutWOB = QHBoxLayout()
        
        self.WOBPlot = pg.PlotWidget()
        self.WOBPlot.setYRange(0, 45)
        self.WOBPlot.setXRange(0,20)
        layoutWOB.addWidget(self.WOBPlot)
        self.graphicsView_adv_WOB.setLayout(layoutWOB)

        
        self.p4 = self.WOBPlot.plotItem
 
        self.p4.showGrid(x = True, y = True, alpha = 0.7)   
        self.WOBCurve = self.p4.plot()
        self.WOBCurve.setPen(pg.mkPen(color="#fff000", width=2))
        self.WOBCurve.getViewBox().invertY(True)
        self.p4.getAxis('right').setLabel('WOB', color='#0000ff')

        #Init the Pressure plot
        layoutPressure = QHBoxLayout()
        
        self.PressurePlot = pg.PlotWidget()
        self.PressurePlot.setYRange(0, 45)
        self.PressurePlot.setXRange(0,5)
        layoutPressure.addWidget(self.PressurePlot)
        self.graphicsView_adv_Pressure.setLayout(layoutPressure)

        
        self.p5 = self.PressurePlot.plotItem
 
        self.p5.showGrid(x = True, y = True, alpha = 0.7)   
        self.PressureCurve = self.p5.plot()
        self.PressureCurve.setPen(pg.mkPen(color="#fff000", width=2))
        self.PressureCurve.getViewBox().invertY(True)
        self.p5.getAxis('right').setLabel('Pressure', color='#0000ff')

        #MSE Plot
        layoutMSE = QHBoxLayout()
        
        self.MSEPlot = pg.PlotWidget()
        self.MSEPlot.setYRange(0, 45)
        self.MSEPlot.setXRange(0,500)
        layoutMSE.addWidget(self.MSEPlot)
        self.graphicsView_adv_MSE.setLayout(layoutMSE)

        
        self.p6 = self.MSEPlot.plotItem
        
        self.p6.showGrid(x = True, y = True, alpha = 0.7)   
        self.MSECurve = self.p6.plot()
        self.MSECurve.setPen(pg.mkPen(color="#fff000", width=2))
        self.MSECurve.getViewBox().invertY(True)
        self.p6.getAxis('right').setLabel('MSE', color='#0000ff')

        #Bit Torque Plot
        layoutBitTorque = QHBoxLayout()
        
        self.BitTorquePlot = pg.PlotWidget()
        self.BitTorquePlot.setYRange(0, 45)
        self.BitTorquePlot.setXRange(0,4)
        layoutBitTorque.addWidget(self.BitTorquePlot)
        self.graphicsView_adv_bitTorque.setLayout(layoutBitTorque)

        
        self.p6 = self.BitTorquePlot.plotItem
        
        self.p6.showGrid(x = True, y = True, alpha = 0.7)   
        self.BitTorqueCurve = self.p6.plot()
        self.BitTorqueCurve.setPen(pg.mkPen(color="#fff000", width=2))
        self.BitTorqueCurve.getViewBox().invertY(True)
        self.p6.getAxis('right').setLabel('Bit Torque', color='#0000ff')

        #Formation Plot
        layoutFormation = QHBoxLayout()
        
        self.FormationPlot = pg.PlotWidget()
        self.FormationPlot.setYRange(0, 45)
        self.FormationPlot.setXRange(0,1)
        layoutFormation.addWidget(self.FormationPlot)
        self.graphicsView_adv_formation.setLayout(layoutFormation)

        
        self.p6 = self.FormationPlot.plotItem
        
        self.p6.showGrid(x = True, y = True, alpha = 0.7)   
        self.FormationCurve = self.p6.plot()
        self.FormationCurve.setPen(pg.mkPen(color="#fff000", width=2))
        self.FormationCurve.getViewBox().invertY(True)
        self.p6.getAxis('right').setLabel('Formation', color='#0000ff')

        #UCS Plot
        layoutUCS = QHBoxLayout()
        
        self.UCSPlot = pg.PlotWidget()
        self.UCSPlot.setYRange(0, 45)
        self.UCSPlot.setXRange(0,175)
        layoutUCS.addWidget(self.UCSPlot)
        self.graphicsView_adv_UCS.setLayout(layoutUCS)

        
        self.p6 = self.UCSPlot.plotItem
        
        self.p6.showGrid(x = True, y = True, alpha = 0.7)   
        self.UCSCurve = self.p6.plot()
        self.UCSCurve.setPen(pg.mkPen(color="#fff000", width=2))
        self.UCSCurve.getViewBox().invertY(True)
        self.p6.getAxis('right').setLabel('UCS', color='#0000ff')

        #RPMWOB Plot
        layoutRPMWOB = QHBoxLayout()
        
        self.RPMWOBPlot = pg.PlotWidget()
        self.RPMWOBPlot.setYRange(0, 1500)
        self.RPMWOBPlot.setXRange(0,15)
        layoutRPMWOB.addWidget(self.RPMWOBPlot)
        self.graphicsView_adv_RPMWOB.setLayout(layoutRPMWOB)

        
        self.p7 = self.RPMWOBPlot.plotItem
        
        self.p7.showGrid(x = True, y = True, alpha = 0.7) 
        
        self.p7.setLabels(left='RPM',bottom="WOB")
        self.RPMWOBCurve = self.p7.plot(symbol="x")
        self.RPMWOBCurve.setPen(pg.mkPen(color="#fff000", width=2))
        self.p7.getAxis('right').setLabel('RPM', color='#0000ff')

        #ROPStep plot
        layoutROPStep = QHBoxLayout()
        
        self.layoutROPPlot = pg.PlotWidget()
        self.layoutROPPlot.setYRange(0, 2)
        self.layoutROPPlot.setXRange(0,100)
        layoutROPStep.addWidget(self.layoutROPPlot)
        self.graphicsView_adv_ROPStep.setLayout(layoutROPStep)

        
        self.p8 = self.layoutROPPlot.plotItem
        
        self.p8.showGrid(x = True, y = True, alpha = 0.7) 
        
        self.p8.setLabels(left='ROP',bottom="Algorithmic steps")
        self.ROPSetCurve = self.p8.plot(symbol="o")
        self.ROPSetCurve.setPen(pg.mkPen(color="#fff000", width=2))
        self.p8.getAxis('right').setLabel('ROP', color='#0000ff')

        #MSEStep plot
        layoutMSEStep = QHBoxLayout()
        
        self.layoutMSEPlot = pg.PlotWidget()
        self.layoutMSEPlot.setYRange(0, 400)
        self.layoutMSEPlot.setXRange(0,100)
        layoutMSEStep.addWidget(self.layoutMSEPlot)
        self.graphicsView_adv_MSEStep.setLayout(layoutMSEStep)

        
        self.p9 = self.layoutMSEPlot.plotItem
        
        self.p9.showGrid(x = True, y = True, alpha = 0.7) 
        
        self.p9.setLabels(left='MSE',bottom="Algorithmic steps")
        self.MSESetCurve = self.p9.plot(symbol="o")
        self.MSESetCurve.setPen(pg.mkPen(color="#fff000", width=2))
        self.p8.getAxis('right').setLabel('MSE', color='#0000ff')


    def updateLabels(self,newFormation,coordinatorProblem,circulationMode,topDriveMode,hoistingMode,rock,wobSetpoint,rpmSetpoint,velocity,TVD,act1,act2,act3,Height,Q,ROP_15s,ROP_3m,Z1,Z2,Z3,sumZ,WOB,Pressure,Torque,RPM,Vibration,MSE,UCS,torqueBit,dExponenet,timeNow):
        WOB = float("{0:.2f}".format(WOB))
        ROP_15s = float("{0:.2f}".format(ROP_15s))
        ROP_3m = float("{0:.2f}".format(ROP_3m))
        MSE = float("{0:.2f}".format(MSE))
        UCS = float("{0:.2f}".format(UCS))
        Q = float("{0:.2f}".format(Q))
        velocity = float("{0:.2f}".format(velocity))
        dExponenet = float("{0:.2f}".format(dExponenet))

        self.label_adv_WOB.setText(str(WOB))
        self.label_adv_Pressure.setText(str(Pressure))
        self.label_adv_torqueMotor.setText(str(Torque))
        self.label_adv_RPM.setText(str(RPM))
        self.label_adv_flowRate.setText(str(Q))
        self.label_adv_ROP15.setText(str(ROP_15s))
        self.label_adv_ROP3.setText(str(ROP_3m))
        self.label_adv_Height.setText(str(Height))
        self.label_adv_Velocity.setText(str(velocity))
        self.label_adv_stepCounter.setText(str(act1))

        self.lcdNumber_WellTVD.display(TVD)
        self.lcdNumber_adv_AvgROP.display(ROP_3m)
        self.lcdNumber_adv_ControllerRPM.display(rpmSetpoint)
        self.lcdNumber_adv_ControllerWOB.display(wobSetpoint)
        self.lcdNumber_adv_Duration.display(timeNow)

        self.dial_adv_MSE.setValue(int(MSE))
        self.dial_adv_ROP.setValue(int(ROP_15s*60))
        self.dial_adv_flowRate.setValue(int(Q*10))
        self.dial_adv_WOB.setValue(int(WOB))
        self.dial_adv_pressure.setValue(int(Pressure*10))
        self.dial_adv_torque.setValue(int(Torque*10))
        self.dial_adv_UCS.setValue(int(UCS))
        self.dial_adv_RPM.setValue(int(RPM))
        self.dial_adv_axialVib.setValue(int(WOB*9.81))

        self.progressBar_adv_MSE.setValue(int(MSE))
        self.progressBar_adv_flowRate.setValue(int(Q*10))
        self.progressBar_adv_WOB.setValue(int(WOB))
        self.progressBar_adv_Pressure.setValue(int(Pressure*10))
        self.progressBar_adv_torque.setValue(int(Torque*10))
        self.progressBar_adv_UCS.setValue(int(UCS))
        self.progressBar_adv_RPM.setValue(int(RPM))
        self.progressBar_adv_axialVib.setValue(int(WOB*9.81))

        if TVD >= 0 and TVD < 7.5:
            if not self.radioButton_12.isChecked():
                self.radioButton_12.toggle()
        elif TVD >= 7.5 and TVD < 15:
            if not self.radioButton_13.isChecked():
                self.radioButton_13.toggle()
        elif TVD >= 15 and TVD < 22.5:
            if not self.radioButton_16.isChecked():
                self.radioButton_16.toggle()
        elif TVD >= 22.5 and TVD < 30:
            if not self.radioButton_10.isChecked():
                self.radioButton_10.toggle()
        elif TVD >= 30 and TVD < 37.5:
            if not self.radioButton_14.isChecked():
                self.radioButton_14.toggle()
        elif TVD >= 37.5 and TVD < 45:
            if not self.radioButton_15.isChecked():
                self.radioButton_15.toggle()
        elif TVD >= 45:
            if not self.radioButton_9.isChecked():
                self.radioButton_9.toggle()
        else:
            pass


        global oldTVD
        global oldWOBset
        global oldRPMset
        if self.initTVD:
            oldTVD = TVD
            oldWOBset = wobSetpoint
            oldRPMset = rpmSetpoint
            self.initTVD = False
        if  oldTVD+0.5 < TVD:
            oldTVD = TVD
            if len(self.RPM) < 100:
                self.RPM.append(RPM)
                self.WOB.append(WOB)
                self.pressure.append(Pressure)
                self.torque.append(Torque)
                self.bit_torque.append(torqueBit)
                self.ROP.append(ROP_15s*60)
                self.TVD.append(TVD)
                self.MSE.append(MSE)
                self.UCS.append(UCS)
            else:
                self.RPM = self.RPM[1:] + [RPM]
                self.WOB = self.WOB[1:] + [WOB]
                self.pressure = self.pressure[1:] + [Pressure]
                self.torque = self.torque[1:] + [Torque]
                self.bit_torque = self.bit_torque[1:] + [torqueBit]
                self.ROP = self.ROP[1:] + [ROP_15s*60]
                self.TVD = self.TVD[1:] + [TVD]
                self.MSE = self.MSE[1:] + [MSE]
                self.UCS = self.UCS[1:] + [UCS]

            self.RPMCurve.setData(self.RPM,self.TVD)
            self.torqueCurve.setData(self.torque,self.TVD)
            self.ROPCurve.setData(self.ROP,self.TVD)
            self.WOBCurve.setData(self.WOB,self.TVD)
            self.PressureCurve.setData(self.pressure,self.TVD)
            self.MSECurve.setData(self.MSE,self.TVD)
            self.BitTorqueCurve.setData(self.bit_torque,self.TVD)
            self.UCSCurve.setData(self.UCS,self.TVD)
        
        if oldWOBset != wobSetpoint or oldRPMset != rpmSetpoint:
            ROPpos=pg.TextItem(text="ROP:" + str(ROP_15s),anchor=(0,1), color=(47, 149, 200))

            ROPpos.setPos(oldWOBset,oldRPMset)
            self.RPMWOBPlot.addItem(ROPpos)
            oldWOBset  = wobSetpoint
            oldRPMset = rpmSetpoint
            if len(self.wobSet) < 100:
                self.wobSet.append(wobSetpoint)
                self.rpmSet.append(rpmSetpoint)
                self.ropSet.append(ROP_15s)
                self.mseSet.append(MSE)
            else:
                self.wobSet = self.wobSet[1:] + [wobSetpoint]
                self.rpmSet = self.rpmSet[1:] + [rpmSetpoint]
                self.ropSet = self.ropSet[1:] + [ROP_15s]
                self.mseSet = self.mseSet[1:] + [MSE]
            
            self.RPMWOBCurve.setData(self.wobSet,self.rpmSet)
            self.ROPSetCurve.setData(self.ropSet)
            self.MSESetCurve.setData(self.mseSet)
    
    
        if hoistingMode == 10:
            self.label_adv_axialVib.setStyleSheet("background-color: red")
        else:
            self.label_adv_axialVib.setStyleSheet("background-color: white")
        if coordinatorProblem == 4:
            self.label_adv_stickSlip.setStyleSheet("background-color: red")
        else:
            self.label_adv_stickSlip.setStyleSheet("background-color: white")

        if coordinatorProblem == 11:
            self.label_adv_overTorque.setStyleSheet("background-color: red")
        else:
            self.label_adv_overTorque.setStyleSheet("background-color: white")

        if coordinatorProblem == 12:
            self.label_adv_TwistOff.setStyleSheet("background-color: red")
        else:
            self.label_adv_TwistOff.setStyleSheet("background-color: white")

        if circulationMode == 3:
            self.label_adv_overpressure.setStyleSheet("background-color: red")
        else:
            self.label_adv_overpressure.setStyleSheet("background-color: white")
        
        if circulationMode == 4:
            self.label_adv_leak.setStyleSheet("background-color: red")
        else:
            self.label_adv_leak.setStyleSheet("background-color: white")

        if newFormation:
            self.label_adv_newForm.setStyleSheet("background-color: green")
        else:
            self.label_adv_newForm.setStyleSheet("background-color: white")



class GUI(QWidget,pyqtdesign.Ui_Form):
    #Init each plot, inherit the desingn produced in QT Desinger
    
    def __init__(self,dataThread,parent=None):
        QWidget.__init__(self,parent)
        self.dataThread = dataThread
        self.dataThread.dataChanged.connect(self.updateGraph)
        
        self.setupUi(self)
        
        
        #List of values that will be displayed in the graph
        self.RPM = []
        self.torque = []
        self.vibration = []
        self.pressure = []
        self.time = []
        self.WOB = []
        self.MSE = []
        self.ROP = []
        #Init the RPM plot
        layoutRPM = QHBoxLayout()
        
        self.RPMPlot = pg.PlotWidget()
        self.RPMPlot.setYRange(0,1500)
        layoutRPM.addWidget(self.RPMPlot)
        self.graphicsView_2.setLayout(layoutRPM) # Places the plot in the graphisView from the desinger

        self.p1 = self.RPMPlot.plotItem
        self.p1.setLabels(left='RPM',bottom="Time[seconds]")
        self.p1.showGrid(x = True, y = True, alpha = 0.7)   
        self.RPMCurve = self.p1.plot()
        self.RPMCurve.setPen(pg.mkPen(color="#fff000", width=2))
        self.p1.getAxis('right').setLabel('RPM', color='#0000ff')

        #Init the Torque plot
        layoutTorque = QHBoxLayout()
        
        self.torquePlot = pg.PlotWidget()
        self.torquePlot.setYRange(0,10)
        layoutTorque.addWidget(self.torquePlot)
        self.graphicsView.setLayout(layoutTorque)

        
        self.p2 = self.torquePlot.plotItem
        self.p2.setLabels(left='Torque',bottom="Time[seconds]")
        self.p2.showGrid(x = True, y = True, alpha = 0.7)   
        self.torqueCurve = self.p2.plot()
        self.torqueCurve.setPen(pg.mkPen(color="#fff000", width=2))
        self.p2.getAxis('right').setLabel('RPM', color='#0000ff')

        #Init the ROP plot
        layoutROP = QHBoxLayout()
        
        self.ROPPlot = pg.PlotWidget()
        self.ROPPlot.setYRange(0,50)
        layoutROP.addWidget(self.ROPPlot)
        self.graphicsView_3.setLayout(layoutROP)

        
        self.p3 = self.ROPPlot.plotItem
        self.p3.setLabels(left='ROP',bottom="Time[seconds]")
        self.p3.showGrid(x = True, y = True, alpha = 0.7)   
        self.ROPCurve = self.p3.plot()
        self.ROPCurve.setPen(pg.mkPen(color="#fff000", width=2))
        self.p3.getAxis('right').setLabel('ROP', color='#0000ff')

        #Init the WOB plot
        layoutWOB = QHBoxLayout()
        
        self.WOBPlot = pg.PlotWidget()
        
        layoutWOB.addWidget(self.WOBPlot)
        self.WOBPlot.setYRange(0,20)
        self.graphicsView_4.setLayout(layoutWOB)

        
        self.p4 = self.WOBPlot.plotItem
        self.p4.setLabels(left='WOB',bottom="Time[seconds]")
        self.p4.showGrid(x = True, y = True, alpha = 0.7)   
        self.WOBCurve = self.p4.plot()
        self.WOBCurve.setPen(pg.mkPen(color="#fff000", width=2))
        self.p4.getAxis('right').setLabel('WOB', color='#0000ff')

        #Init the Pressure plot
        layoutPressure = QHBoxLayout()
        
        self.PressurePlot = pg.PlotWidget()
        self.PressurePlot.setYRange(0,5)
        layoutPressure.addWidget(self.PressurePlot)
        self.graphicsView_5.setLayout(layoutPressure)

        
        self.p5 = self.PressurePlot.plotItem
        self.p5.setLabels(left='Pressure',bottom="Time[seconds]")
        self.p5.showGrid(x = True, y = True, alpha = 0.7)   
        self.PressureCurve = self.p5.plot()
        self.PressureCurve.setPen(pg.mkPen(color="#fff000", width=2))
        self.p5.getAxis('right').setLabel('Pressure', color='#0000ff')

        #MSE Plot
        layoutMSE = QHBoxLayout()
        
        self.MSEPlot = pg.PlotWidget()
        layoutMSE.addWidget(self.MSEPlot)
        self.MSEPlot.setYRange(0,500)
        self.graphicsView_6.setLayout(layoutMSE)

        
        self.p6 = self.MSEPlot.plotItem
        self.p6.setLabels(left='MSE',bottom="Time[seconds]")
        self.p6.showGrid(x = True, y = True, alpha = 0.7)   
        self.MSECurve = self.p6.plot()
        self.MSECurve.setPen(pg.mkPen(color="#fff000", width=2))
        self.p6.getAxis('right').setLabel('MSE', color='#0000ff')

    #When called, push new data in the list of data and updates graph
    def updateGraph(self,newFormation,coordinatorProblem,circulationMode,topDriveMode,hoistingMode,rock,wobSetpoint,rpmSetpoint,velocity,TVD,act1,act2,act,Height,Q,ROP_15s,ROP_3m,Z1,Z2,Z3,sumZ,WOB,Pressure,Torque,RPM,Vibration,MSE,UCS,torqueBit,dExponenet,timeNow):
        if len(self.RPM) < 50:
            self.RPM.append(RPM)
            self.WOB.append(WOB)
            self.pressure.append(Pressure)
            self.torque.append(Torque)
            self.ROP.append(ROP_15s*60)
            self.time.append(timeNow)
            self.MSE.append(MSE)
        else:
            self.RPM = self.RPM[1:] + [RPM]
            self.WOB = self.WOB[1:] + [WOB]
            self.pressure = self.pressure[1:] + [Pressure]
            self.torque = self.torque[1:] + [Torque]
            self.ROP = self.ROP[1:] + [ROP_15s*60]
            self.MSE = self.MSE[1:] + [MSE]
            self.time = self.time[1:] + [timeNow]

        self.RPMCurve.setData(self.time,self.RPM)
        self.torqueCurve.setData(self.time,self.torque)
        self.ROPCurve.setData(self.time, self.ROP)
        self.WOBCurve.setData(self.time, self.WOB)
        self.PressureCurve.setData(self.time,self.pressure)
        self.MSECurve.setData(self.time,self.MSE)
        
        


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ContSys = ControlUI()
    ContSys.show()
    sys.exit(app.exec_())
   
  
