
import pandas as pd  
import numpy as np  
import math
import os
import pickle
from sklearn.model_selection import train_test_split  
from sklearn.svm import SVC
from sklearn.metrics import classification_report, confusion_matrix  
from sklearn.linear_model import SGDClassifier
from sklearn.neighbors.nearest_centroid import NearestCentroid


class Classify():
    def __init__(self):
        self.sensorData = pd.DataFrame(columns=['Loadcell_z1'])
        self.features = {
            "z1" : pd.DataFrame(columns=['Mean','Median', 'STD', 'Var','RMS','MAD','p25','p75','IQR','Skewness', 'Kurtosis','CF','SF','IF','ET','MF'])
        }
        
        self.WINDOW_SIZE = 322
        self.sizeNow = 0
        self.predicedLabel = "No_prediction"
        self.model = pickle.load(open('TrainedSVM.sav', 'rb'))

    def predict(self,z1):
        if self.sizeNow < self.WINDOW_SIZE:
            #print(self.sizeNow)
            self.sensorData.loc[self.sizeNow,"Loadcell_z1"] = z1
            self.sizeNow += 1
        else: 
            self.sizeNow = 0
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

        return self.predicedLabel


    def predictModel(self):
        print(self.features["z1"].head())
        if self.features["z1"].isnull().values.any():
            return self.predicedLabel
        else:

            return self.model.predict(self.features["z1"])

    def square(self,x):
        return x**2

    def Mean(self,sliceDF):
        print(sliceDF.shape)
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
        self.features["z1"].loc[0,"Var"] = calcVal.loc["Loadcell_z1"]

    def RMS(self,sliceDF):
        calcVal = sliceDF.apply(self.square)
        calcVal = math.sqrt((sliceDF.sum())/self.WINDOW_SIZE)

        self.features["z1"].loc[0,"RMS"] = calcVal

    def MAD(self,sliceDF):
        calcVal = sliceDF.mad()

        self.features["z1"].loc[0,"MAD"] = calcVal.loc["Loadcell_z1"]

    def p25(self,sliceDF):
        calcVal = sliceDF.quantile(.25)
        self.features["z1"].loc[0,"p25"] = calcVal.loc["Loadcell_z1"]

    def p75(self,sliceDF):
        calcVal = sliceDF.quantile(.75)
        self.features["z1"].loc[0,"p75"] = calcVal.loc["Loadcell_z1"]

    def IQR(self,sliceDF):
        calcVal = (sliceDF.quantile(.75) - sliceDF.quantile(.25))
        self.features["z1"].loc[0,"IQR"] = calcVal.loc["Loadcell_z1"]

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
        calcVal = sliceDF.abs()
        calcVal = calcVal.apply(np.sqrt)
        calcVal = (calcVal.mean())**2
        self.features["z1"].loc[0,"ET"] = calcVal.loc["Loadcell_z1"]
        

    def MF(self,sliceDF):
        maxVal = sliceDF.max()
        calcVal = sliceDF.abs()
        calcVal = calcVal.apply(np.sqrt)
        calcVal =  (calcVal.mean())**2
        calcVal = (maxVal / calcVal)
        self.features["z1"].loc[0,"MF"] = calcVal.loc["Loadcell_z1"]
