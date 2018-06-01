
import pandas as pd  
import numpy as np  
import math
import os
from statsmodels import robust
import time
from threading import Thread
import pickle
from sklearn.model_selection import train_test_split  
from sklearn.svm import SVC
from sklearn.metrics import classification_report, confusion_matrix  
from sklearn.linear_model import SGDClassifier
from sklearn.neighbors.nearest_centroid import NearestCentroid

new_inc_data = False
class Classify(Thread):
    def __init__(self,ArduinoHoistingData):
        Thread.__init__(self,daemon=True)
        self.sensorData = pd.DataFrame(columns=['Loadcell_z1'])
        self.features = {
            "z1" : pd.DataFrame(columns=['Mean','Median', 'STD', 'Var','RMS','MAD','p25','p75','IQR','Skewness', 'Kurtosis','CF','SF','IF','ET','MF'])
        }
        
        self.WINDOW_SIZE = 322
        self.sizeNow = 0
        self.predicedLabel = "No_prediction"
        self.model = pickle.load(open('TrainedSVM.sav', 'rb'))
        self.ArduinoHoistingData = ArduinoHoistingData

    def run(self):
        while True:
            global new_inc_data
            if new_inc_data:
                z1 = self.ArduinoHoistingData.getHoistingSensorData()["z1"]
                if self.sizeNow < self.WINDOW_SIZE:
                    #print(self.sizeNow)
                    self.sensorData.loc[self.sizeNow,"Loadcell_z1"] = z1
                    self.sizeNow += 1
                else: 
                    self.sizeNow = 0
                    self.sensorData = (self.sensorData-self.sensorData.min())/(self.sensorData.max()-self.sensorData.min())
                    self.Mean(self.sensorData)
                    self.Median(self.sensorData)
                    self.STD(self.sensorData)
                    self.Var(self.sensorData)
                    self.RMS(self.sensorData)
                    self.MAD(self.sensorData)
                    self.p25(self.sensorData)
                    self.p75(self.sensorData)
                    self.IQR(self.sensorData)
                    self.Skewness(self.sensorData)
                    self.Kurtosis(self.sensorData)
                    self.CF(self.sensorData)
                    self.SF(self.sensorData)
                    self.IF(self.sensorData)
                    self.ET(self.sensorData)
                    self.MF(self.sensorData)
                    self.predicedLabel = self.predictModel()
                new_inc_data = False

            time.sleep(0.01)
    def predictModel(self):
        #print(self.features["z1"].head())
        if self.features["z1"].isnull().values.any():      
            return self.predicedLabel     
        else:
            return self.model.predict(self.features["z1"])
            
    def square(self,x):
        return x**2

    def Mean(self,sliceDF):

        calcVal = sliceDF.mean()
        
        self.features["z1"].loc[0,"Mean"] = calcVal.loc["Loadcell_z1"]
        
    def Median(self,sliceDF):
        calcVal = sliceDF.median()
        self.features["z1"].loc[0,"Median"] = calcVal.loc["Loadcell_z1"]
        

    def STD(self,sliceDF):
        calcVal = sliceDF.std(ddof=0)
        self.features["z1"].loc[0,"STD"] = calcVal.loc["Loadcell_z1"]

    def Var(self,sliceDF):
        calcVal = sliceDF.var()
        #print(calcVal)
        self.features["z1"].loc[0,"Var"] = calcVal.loc["Loadcell_z1"]

    def RMS(self,sliceDF):
        calcVal = sliceDF.apply(self.square)
        calcVal = math.sqrt((sliceDF.sum())/self.WINDOW_SIZE)
        
        self.features["z1"].loc[0,"RMS"] = calcVal

    def MAD(self,sliceDF):
        matrix = sliceDF.values
        arr = np.ma.array(matrix).compressed() 
        med = np.median(matrix)
        calcVal = np.median(np.abs(arr - med))
        try:
            self.features["z1"].loc[0,"MAD"] = calcVal
        except:
            self.features["z1"].loc[0,"MAD"] = 0

    def p25(self,sliceDF):
        try:
            matrix = sliceDF.values
            calcVal = np.percentile(matrix, 25)
            self.features["z1"].loc[0,"p25"] = calcVal
        except:
            self.features["z1"].loc[0,"p25"] = 0

    def p75(self,sliceDF):
        try:
            matrix = sliceDF.values
            calcVal = np.percentile(matrix, 75)
            self.features["z1"].loc[0,"p75"] = calcVal
        except:
            self.features["z1"].loc[0,"p75"] = 0
    def IQR(self,sliceDF):
        try:
            matrix = sliceDF.values
            calcVal = (np.percentile(matrix, 75) - np.percentile(matrix, 25))
            self.features["z1"].loc[0,"IQR"] = calcVal.loc["Loadcell_z1"]
        except:
            self.features["z1"].loc[0,"IQR"] = 0
    def Skewness(self,sliceDF):
        calcVal = sliceDF.skew()
        self.features["z1"].loc[0,"Skewness"] = calcVal.loc["Loadcell_z1"]

    def Kurtosis(self,sliceDF):
        calcVal = sliceDF.kurtosis()
        self.features["z1"].loc[0,"Kurtosis"] = calcVal.loc["Loadcell_z1"]


    def CF(self,sliceDF):
        try:
            calcVal = (sliceDF.max() / (math.sqrt(((sliceDF.sum())**2)/self.WINDOW_SIZE)))
        except:
            calcVal = 0
        self.features["z1"].loc[0,"CF"] = calcVal.loc["Loadcell_z1"]

    def SF(self,sliceDF):
        calcVal = sliceDF.apply(self.square)
        calcVal = (math.sqrt((sliceDF.sum())/self.WINDOW_SIZE)) / sliceDF.mean()
        self.features["z1"].loc[0,"SF"] = calcVal.loc["Loadcell_z1"]

    def IF(self,sliceDF):
        calcVal = sliceDF.abs()
        calcVal = sliceDF.max()/ calcVal.mean()
        self.features["z1"].loc[0,"IF"] = calcVal.loc["Loadcell_z1"]

    def ET(self,sliceDF):
        matrix = sliceDF.values
        arr = np.ma.array(matrix).compressed() 
        square_roots = [x ** 0.5 for x in arr]
        #print(square_roots)
        calcVal = (np.mean(square_roots))**2
        self.features["z1"].loc[0,"ET"] = calcVal
     

    def MF(self,sliceDF):
        matrix = sliceDF.values
        maxVal = np.amax(matrix)
        arr = np.ma.array(matrix).compressed() 
        square_roots = [x ** 0.5 for x in arr]
        #print(square_roots)
        calcVal = (np.mean(square_roots))**2
        calcVal = (maxVal / calcVal)
        self.features["z1"].loc[0,"MF"] = calcVal
    