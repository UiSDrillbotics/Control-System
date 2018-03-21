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

rotationSystem = Rotation.Rotation(t3)
hoistingSystem = Hoisting.Hoisting(t1)



#Gets data and triggers the plot
class GetData(QThread):
    dataChanged = pyqtSignal(float, float, float, float,float,float,float)
    def __init__(self, parent=None):
        QThread.__init__(self, parent)
        self.t = QTime()

    def __del__(self):  # part of the standard format of a QThread
        self.wait()
    def run(self):  
        self.t.start()
        while True:
            hSensorData = t1.getHoistingSensorData()
            cSensorData = t2.getCirculationSensorData()
            rSensorData = t3.getRotationSensorData()
            WOB = hSensorData["WOB"]
            Pressure = cSensorData["pressure"]
            Torque = rSensorData["torque"]
            RPM = rSensorData["RPM"]
            Vibration = rSensorData["vibration"]
            MSE = 0.35*((WOB/1.125) + (120*3.14*RPM*Torque)/(1.125*1)) #Needs reconfiguration ROP and AB

            timeNow = float(self.t.elapsed())/1000
            time.sleep(0.1)
            self.dataChanged.emit(WOB,Pressure,Torque,RPM,Vibration,MSE,timeNow) #Triggers and updates the plot and labels


class ControlUI(QWidget,controls.Ui_C):
    def __init__(self, parent=None):
        QWidget.__init__(self,parent)
        self.setupUi(self)
        self.dataThread = GetData(self)
        self.dataThread.start()
        self.dataThread.dataChanged.connect(self.updateLabels)
        self.pushButton_Advanced_UI.clicked.connect(self.showAdvancedUI)
        self.pushButton_OpenPorts.clicked.connect(self.getComPorts)
        self.pushButton_RPM_Enter.clicked.connect(self.setRPM)
        self.pushButton_Stop_Rotation.clicked.connect(self.stopRotation)
        self.pushButton_Hoistiong_Enter.clicked.connect(self.moveHoisting)
        self.pushButton_Calibrate.clicked.connect(self.calibrate)
        self.pushButton_Open_Brake.clicked.connect(self.openBrake)
        self.pushButton_Stop_Brake.clicked.connect(self.closeBreak)
        self.pushButton_Stop_Hoisting.clicked.connect(self.stopHoisting)
        self.pushButton_WOB_Ctrl.clicked.connect(self.wobControl)
        self.pushButton_Telementary.clicked.connect(self.telementary)
        self.pushButton_PID_Enter.clicked.connect(self.setPIDController)
        self.pushButton_Reset_Stepper_Pos.clicked.connect(self.resetSteppers)

        if sys.platform.startswith('win'):
            import serial.tools.list_ports_windows
            ports = serial.tools.list_ports_windows.comports()
        else:
            import serial.tools.list_ports_osx
            ports = serial.tools.list_ports_osx.comports()

        connectedPorts = []
        for element in ports:
            connectedPorts.append(str(element.device))
        #Populates the comboboxes with connected ports
        self.comboBox_Rotation.addItems(connectedPorts)
        self.comboBox_Circulation.addItems(connectedPorts)
        self.comboBox_Hoisting.addItems(connectedPorts)

        #Populates Accuator combobox
        self.comboBox_Actuator.addItems(["1","2","3","All"])

        #Populates the Driection combobox
        self.comboBox_Direction.addItems(["Raise", "Lower"])


    def showAdvancedUI(self):
        self.window = QtWidgets.QWidget()
       
        self.ui = GUI(self.dataThread)
        self.ui.show()

    def getComPorts(self):
        
        hoistingPort = self.comboBox_Rotation.currentText()
        circulationPort = self.comboBox_Circulation.currentText()
        rotationPort = self.comboBox_Rotation.currentText()
        t1.setSerialPort(hoistingPort)
        t2.setSerialPort(circulationPort)
        t3.setSerialPort(rotationPort)
        #Start each thread
        t1.start()
        t2.start()
        t3.start()
        self.pushButton_OpenPorts.setDisabled(True)

    def setRPM(self):
        rpm = self.spinBox_RPM.value()
        if int(rpm) == 0:
            self.label_Rotating.setStyleSheet("background-color: white")
            self.label_Stopped_2.setStyleSheet("background-color: green")
        else:
            self.label_Rotating.setStyleSheet("background-color: green")
            self.label_Stopped_2.setStyleSheet("background-color: white")
        rotationSystem.setRPM(int(rpm))

    def stopRotation(self):
        self.label_Rotating.setStyleSheet("background-color: white")
        self.label_Stopped_2.setStyleSheet("background-color: green")
        rotationSystem.setRPM(0)
        
    def moveHoisting(self):
        actuator = self.comboBox_Actuator.currentText()
        
        distance = str(self.doubleSpinBox_Distance.value())
        stepperDelay = str(self.doubleSpinBox_Stepper_Delay.value())

        direction = self.comboBox_Direction.currentText()
        
        hoistingSystem.move(distance,direction,stepperDelay,actuator)

    def calibrate(self):
        hoistingSystem.calibrate()

    def openBrake(self):
        hoistingSystem.brake(2)

    def closeBreak(self):
        hoistingSystem.brake(1)

    def stopHoisting(self):
        hoistingSystem.stop()

    def wobControl(self):
        hoistingSystem.wob()

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


    def updateLabels(self,WOB,Pressure,Torque,RPM,Vibration,MSE,timeNow):
        self.label_WOB.setText(str(WOB))
        self.label_Pressure.setText(str(Pressure))
        self.label_Torque.setText(str(Torque))
        self.label_RPM.setText(str(RPM))
        self.label_Z3.setText(str(Vibration))
        self.label_MSE.setText(str(MSE))
    
    


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
    def updateGraph(self,WOB,Pressure,Torque,RPM,Vibration,MSE,timeNow):
        if len(self.RPM) < 200:
            self.RPM.append(RPM)
            self.WOB.append(WOB)
            self.pressure.append(Pressure)
            self.torque.append(Torque)
            self.vibration.append(Vibration)
            self.time.append(timeNow)
            self.MSE.append(MSE)
        else:
            self.RPM = self.RPM[1:] + [RPM]
            self.WOB = self.WOB[1:] + [WOB]
            self.pressure = self.pressure[1:] + [Pressure]
            self.torque = self.torque[1:] + [Torque]
            self.vibration = self.vibration[1:] + [Vibration]
            self.MSE = self.MSE[1:] + [MSE]
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
   
  
    
