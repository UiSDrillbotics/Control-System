from dataManager import DataManager
import time
from threading import Thread


new_inc_data = False

class Database(Thread):
    def __init__(self, ArduinoHoistingData, ArduinoRotationData, ArduinoCirculationData, Hoistingsystem, CirculationSystem, RotaitonSystem):
        Thread.__init__(self,daemon=True)
        self.ArduinoHoistingData = ArduinoHoistingData
        self.ArduinoRotationData = ArduinoRotationData
        self.ArduinoCirculationData = ArduinoCirculationData
        self.HoistingSystem = Hoistingsystem
        self.CirculationSystem = CirculationSystem
        self.RotaitonSystem  = RotaitonSystem
        self.dataManager = DataManager()

        
        self.dataBaseBuffer = {
            "RPM"  :0,
            "Top_Drive_Torque" :0,
            "Pressure" : 0,
            "Loadcell_z1" : 0,
            "Loadcell_z2" : 0,
            "Loadcell_z3" :0,
            "ROP_15s_avg" : 0,
            "ROP_3m_avg" : 0,
            "Flow_Rate" : 0,
            "MSE" : 0,
            "UCS" : 0,
            "TVD" : 0,
            "Bit_Torque" : 0,
            "WOB" : 0,
            "d_exponent" : 0,
            "act1" : 0,
            "act2" : 0,
            "act3" : 0,
            "Velocity" : 0,
            "Height_Sensor": 0
        }
    def initDb(self):

        self.dataManager.initDatabase()      
        self.dataManager.connect('logger')
        if self.dataManager.isConnected == True:
            for key,_ in self.dataBaseBuffer.items():
                self.dataManager.createTable(key)



    def run(self):
        
        while True:
            global new_inc_data
            if new_inc_data:
                #Get the dictonarys from the arduinoData module
                hSensorData = self.ArduinoHoistingData.getHoistingSensorData()
                cSensorData = self.ArduinoCirculationData.getCirculationSensorData()
                rSensorData = self.ArduinoRotationData.getRotationSensorData()
                #Init each variable with the correspondable dictonary value
                RPM = rSensorData["measuredRPM"]
                if RPM <=0:
                    RPM = 0
                torqueMotor = rSensorData["torqueMotor"]
                #torqueSensor = rSensorData["torqueSensor"]
                pressure = cSensorData["pressure"]


                Z1 = hSensorData["z1"]
                Z2 = hSensorData["z2"]
                Z3 = hSensorData["z3"]
                #sumZ = hSensorData["sumZ"]
                ROP_15s = (self.HoistingSystem.calcROP15s())/15

                ROP_3m = (self.HoistingSystem.calcROP3m())/180
                # 3. degree polynomial Q = (-72.831*pressure**3) + (499.29*pressure**2) - (1124.5*pressure) + 842.08
                # 2. degree polynomial Q = (-19.109*pressure**2) + (95.095*pressure) - 106.56
                
                if self.CirculationSystem.pumpOn == False:
                    Q = 0

                else:
                    Q = (3.7417*pressure) + 0.7122
                
                MSE = self.HoistingSystem.calcMSE() 
                UCS = self.HoistingSystem.calcUCS()
                TVD = hSensorData["TVD"]/1000
                torqueBit = 0 # must be updated here, and updated in hoistingSystem
                WOB = self.HoistingSystem.getWOB()
                dExponenet = self.HoistingSystem.calcDexponent()
                #Height = hSensorData["heightSensor"]

                act1 = hSensorData["stepperArduinoPos1"]
                act2 = hSensorData["stepperArduinoPos2"]
                act3 = hSensorData["stepperArduinoPos3"]
                
                #vibration = 0
                velocity = self.HoistingSystem.velocity()


                self.dataManager.pushIntoSqlBuffer('RPM',RPM)
                self.dataManager.pushIntoSqlBuffer('WOB',WOB)
                self.dataManager.pushIntoSqlBuffer('ROP_15s_avg',ROP_15s)
                self.dataManager.pushIntoSqlBuffer('ROP_3m_avg',ROP_3m)
                self.dataManager.pushIntoSqlBuffer('Velocity',velocity)
                self.dataManager.pushIntoSqlBuffer('Top_Drive_Torque',torqueMotor)
                self.dataManager.pushIntoSqlBuffer('Bit_Torque',torqueBit)
                self.dataManager.pushIntoSqlBuffer('Pressure',pressure)
                self.dataManager.pushIntoSqlBuffer('Flow_Rate', Q)
                self.dataManager.pushIntoSqlBuffer('Height_Sensor',0)
                self.dataManager.pushIntoSqlBuffer('Act_Stepcounter_1',act1)
                self.dataManager.pushIntoSqlBuffer('Act_Stepcounter_2',act2)
                self.dataManager.pushIntoSqlBuffer('Act_Stepcounter_3',act3)
                self.dataManager.pushIntoSqlBuffer('Loadcell_z1',Z1)
                self.dataManager.pushIntoSqlBuffer('Loadcell_z2',Z2)
                self.dataManager.pushIntoSqlBuffer('Loadcell_z3',Z3)
                self.dataManager.pushIntoSqlBuffer('TVD',TVD)
                self.dataManager.pushIntoSqlBuffer('MSE',MSE)
                self.dataManager.pushIntoSqlBuffer('UCS',UCS)
                self.dataManager.pushIntoSqlBuffer('d_exponent',dExponenet)
                
                new_inc_data = False
            time.sleep(0.01)