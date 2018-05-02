import Circulation
import Hoisting
import threading
import ArduinoData
import time
import sys
import pyqtdesign
import controls
import pyqtgraph as pg
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QThread,pyqtSignal,QTime
from PyQt5.QtWidgets import QApplication, QMainWindow
import Drillbotics2018

import Hoisting
import Circulation
import Rotation
import Coordinator



#Lock for each arduino data storage
hoistigLock = threading.Lock()
circulationLock = threading.Lock()
rotationLock = threading.Lock()

#Init each thread for reading arduino data
t1 = ArduinoData.HoistingData(hoistigLock)
t2 = ArduinoData.CirculationData(circulationLock)
t3 = ArduinoData.RotationData(rotationLock)


hoistingSystem = Hoisting.Hoisting(t1,t3)
circulationSystem = Circulation.Circulation(t2)
rotationSystem = Rotation.Rotation(t3)

coordinationSystem = Coordinator.Coordination(hoistingSystem,rotationSystem,circulationSystem,t1,t3,t2)


#Gets data and triggers the plot
class GetData(QThread):
    dataChanged = pyqtSignal(float,float,float,float,float,float,float,float,float, float, float, float,float,float,float,float,float,float,float,float,float)
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


            Z1 = hSensorData["z1"]
            Z2 = hSensorData["z2"]
            Z3 = hSensorData["z3"]
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
            torqueBit = 0 # must be updated here, and updated in hoistingSystem
            WOB = hoistingSystem.getWOB()
            dExponenet = hoistingSystem.calcDexponent()
            Height = hSensorData["heightSensor"]

            act1 = hSensorData["stepperArduinoPos1"]
            act2 = hSensorData["stepperArduinoPos2"]
            act3 = hSensorData["stepperArduinoPos3"]
            
            vibration = 0
            

            timeNow = float(self.t.elapsed())/1000
            
            #Sleeps to not overload the system
            time.sleep(0.1)
            #Sends the new data to the chart and labels in the HMI
            self.dataChanged.emit(act1,act2,act3,Height,Q,ROP_15s,ROP_3m,Z1,Z2,Z3,sumZ,WOB,pressure,torqueMotor,RPM,vibration,MSE,UCS,torqueBit,dExponenet,timeNow) #Triggers and updates the plot and labels


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
        else:
            import serial.tools.list_ports_osx
            ports = serial.tools.list_ports_osx.comports()
        #Stores the connected ports for the HMI to show
        connectedPorts = []
        for element in ports:
            connectedPorts.append(str(element.device))
        #Populates the comboboxes with connected ports
        self.comboBox_Rotation.addItems(["COM5"]) # should be (connectedPorts) by default
        self.comboBox_Circulation.addItems(["COM4"]) # should be (connectedPorts) by default
        self.comboBox_Hoisting.addItems(["COM3"]) # should be (connectedPorts) by default

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
        self.pushButton_OpenPorts.setDisabled(True)
    #Method for reading the selected RPM and send it to the Rotation module
    def setRPM(self):
        rpm = self.doubleSpinBox_RPM.value()
        #if int(rpm) == 0:
            #self.label_Rotating.setStyleSheet("background-color: white")
            #self.label_Stopped_2.setStyleSheet("background-color: green")
        #else:
            #self.label_Rotating.setStyleSheet("background-color: green")
            #self.label_Stopped_2.setStyleSheet("background-color: white")
        rotationSystem.setRPM(int(rpm))

    def stopRotation(self):
        #self.label_Rotating.setStyleSheet("background-color: white")
        #self.label_Stopped_2.setStyleSheet("background-color: green")
        rotationSystem.setRPM(0)
        
    def moveHoisting(self):
        actuator = self.comboBox_Actuator.currentText()
        distance = str(int(self.doubleSpinBox_Distance.value()))
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
 
    
    def updateLabels(self,act1,act2,act3,Height,Q,ROP_15s,ROP_3m,Z1,Z2,Z3,sumZ,WOB,Pressure,Torque,RPM,Vibration,MSE,UCS,torqueBit,dExponenet,timeNow):
        WOB = float("{0:.2f}".format(WOB))
        self.label_WOB.setText(str(WOB))
        self.label_Pressure.setText(str(Pressure))
        self.label_Topdrive_Torque.setText(str(Torque))
        self.label_RPM.setText(str(RPM))
        self.label_Axial_Vibration.setText(str(Vibration))
        self.label_MSE.setText(str(Q))
        self.label_LoadCell_Z1.setText(str(int(Z1)))
        self.label_LoadCell_Z2.setText(str(int(Z2)))
        self.label_LoadCell_Z3.setText(str(int(Z3)))
        self.label_Hookload_SumZ.setText(str(int(sumZ)))
        self.label_ROP15.setText(str(ROP_15s))
        self.label_ROP3.setText(str(ROP_3m))
        self.label_Height_Sensor.setText(str(Height))
        self.label_UCS.setText(str(UCS))
        #self.label_MSE.setText(str(MSE))
        self.label_d_exponent.setText(str(dExponenet))
        self.label_Act1StepCount.setText(str(act1))
        self.label_Act1StepCount.setText(str(act2))
        self.label_Act1StepCount.setText(str(act3))
    
    


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

        #Init the RPM plot
        layoutRPM = QHBoxLayout()
        
        self.RPMPlot = pg.PlotWidget()
        layoutRPM.addWidget(self.RPMPlot)
        self.graphicsView_2.setLayout(layoutRPM) # Places the plot in the graphisView from the desinger

        
        self.p1 = self.RPMPlot.plotItem
        self.p1.setLabels(left='RPM',bottom="Time[seconds]")
        self.p1.showGrid(x = True, y = True, alpha = 0.4)   
        self.RPMCurve = self.p1.plot()
        self.RPMCurve.setPen(pg.mkPen(color="#fff000", width=2))
        self.p1.getAxis('right').setLabel('RPM', color='#0000ff')

        #Init the Torque plot
        layoutTorque = QHBoxLayout()
        
        self.torquePlot = pg.PlotWidget()
        layoutTorque.addWidget(self.torquePlot)
        self.graphicsView.setLayout(layoutTorque)

        
        self.p2 = self.torquePlot.plotItem
        self.p2.setLabels(left='Torque',bottom="Time[seconds]")
        self.p2.showGrid(x = True, y = True, alpha = 0.4)   
        self.torqueCurve = self.p2.plot()
        self.torqueCurve.setPen(pg.mkPen(color="#fff000", width=2))
        self.p2.getAxis('right').setLabel('RPM', color='#0000ff')

        #Init the Vibration plot
        layoutVibration = QHBoxLayout()
        
        self.vibrationPlot = pg.PlotWidget()
        layoutVibration.addWidget(self.vibrationPlot)
        self.graphicsView_3.setLayout(layoutVibration)

        
        self.p3 = self.vibrationPlot.plotItem
        self.p3.setLabels(left='Vibration',bottom="Time[seconds]")
        self.p3.showGrid(x = True, y = True, alpha = 0.4)   
        self.vibrationCurve = self.p3.plot()
        self.vibrationCurve.setPen(pg.mkPen(color="#fff000", width=2))
        self.p3.getAxis('right').setLabel('Vibration', color='#0000ff')

        #Init the WOB plot
        layoutWOB = QHBoxLayout()
        
        self.WOBPlot = pg.PlotWidget()
        layoutWOB.addWidget(self.WOBPlot)
        self.graphicsView_4.setLayout(layoutWOB)

        
        self.p4 = self.WOBPlot.plotItem
        self.p4.setLabels(left='WOB',bottom="Time[seconds]")
        self.p4.showGrid(x = True, y = True, alpha = 0.4)   
        self.WOBCurve = self.p4.plot()
        self.WOBCurve.setPen(pg.mkPen(color="#fff000", width=2))
        self.p4.getAxis('right').setLabel('WOB', color='#0000ff')

        #Init the Pressure plot
        layoutPressure = QHBoxLayout()
        
        self.PressurePlot = pg.PlotWidget()
        layoutPressure.addWidget(self.PressurePlot)
        self.graphicsView_5.setLayout(layoutPressure)

        
        self.p5 = self.PressurePlot.plotItem
        self.p5.setLabels(left='Pressure',bottom="Time[seconds]")
        self.p5.showGrid(x = True, y = True, alpha = 0.4)   
        self.PressureCurve = self.p5.plot()
        self.PressureCurve.setPen(pg.mkPen(color="#fff000", width=2))
        self.p5.getAxis('right').setLabel('Pressure', color='#0000ff')

        #MSE Plot
        layoutMSE = QHBoxLayout()
        
        self.MSEPlot = pg.PlotWidget()
        layoutMSE.addWidget(self.MSEPlot)
        self.graphicsView_6.setLayout(layoutMSE)

        
        self.p6 = self.MSEPlot.plotItem
        self.p6.setLabels(left='MSE',bottom="Time[seconds]")
        self.p6.showGrid(x = True, y = True, alpha = 0.4)   
        self.MSECurve = self.p6.plot()
        self.MSECurve.setPen(pg.mkPen(color="#fff000", width=2))
        self.p6.getAxis('right').setLabel('MSE', color='#0000ff')

    #When called, push new data in the list of data and updates graph
    def updateGraph(self,act1,act2,act,Height,Q,ROP_15s,ROP_3m,Z1,Z2,Z3,sumZ,WOB,Pressure,Torque,RPM,Vibration,MSE,UCS,torqueBit,dExponenet,timeNow):
        if len(self.RPM) < 200:
            self.RPM.append(RPM)
            self.WOB.append(WOB)
            self.pressure.append(Pressure)
            self.torque.append(Torque)
            self.vibration.append(Vibration)
            self.time.append(timeNow)
            self.MSE.append(Q)
        else:
            self.RPM = self.RPM[1:] + [RPM]
            self.WOB = self.WOB[1:] + [WOB]
            self.pressure = self.pressure[1:] + [Pressure]
            self.torque = self.torque[1:] + [Torque]
            self.vibration = self.vibration[1:] + [Vibration]
            self.MSE = self.MSE[1:] + [Q]
            self.time = self.time[1:] + [timeNow]

        self.RPMCurve.setData(self.time,self.RPM)
        self.torqueCurve.setData(self.time,self.torque)
        self.vibrationCurve.setData(self.time, self.vibration)
        self.WOBCurve.setData(self.time, self.WOB)
        self.PressureCurve.setData(self.time,self.pressure)
        self.MSECurve.setData(self.time,self.MSE)
        
        

if __name__ == "__main__":
    app = QApplication(sys.argv)
    ContSys = ControlUI()
    ContSys.show()
    sys.exit(app.exec_())
   
  
    
