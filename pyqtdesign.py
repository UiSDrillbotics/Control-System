# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'design.ui'
#
# Created by: PyQt5 UI code generator 5.6
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets
import sys

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Graphs")
        Form.resize(1507, 1136)
        self.gridLayout = QtWidgets.QGridLayout(Form)
        self.gridLayout.setObjectName("gridLayout")



        self.graphicsView = QtWidgets.QGraphicsView(Form)
        self.graphicsView.setObjectName("graphicsView")
        self.gridLayout.addWidget(self.graphicsView, 0, 0, 1, 1)
        self.graphicsView_2 = QtWidgets.QGraphicsView(Form)
        self.graphicsView_2.setObjectName("graphicsView_2")
        self.gridLayout.addWidget(self.graphicsView_2, 1, 0, 1, 1)
  

        self.graphicsView_3 = QtWidgets.QGraphicsView(Form)
        self.graphicsView_3.setObjectName("graphicsView_3")
        self.gridLayout.addWidget(self.graphicsView_3, 2, 0, 1, 1)
       

        self.graphicsView_4 = QtWidgets.QGraphicsView(Form)
        self.graphicsView_4.setObjectName("graphicsView_4")
        self.gridLayout.addWidget(self.graphicsView_4, 0, 1, 1, 1)


        
        self.graphicsView_5 = QtWidgets.QGraphicsView(Form)
        self.graphicsView_5.setObjectName("graphicsView_5")
        self.gridLayout.addWidget(self.graphicsView_5, 1, 1, 1, 1)

        self.graphicsView_6 = QtWidgets.QGraphicsView(Form)
        self.graphicsView_6.setObjectName("graphicsView_6")
        self.gridLayout.addWidget(self.graphicsView_6, 2,1, 1, 1)


        #self.graphicsView_7 = QtWidgets.QGraphicsView(Form)
        #self.graphicsView_7.setObjectName("graphicsView_7")
        #self.gridLayout.addWidget(self.graphicsView_7, 0,2, 3, 2)


   


        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Graphs", "Graphs"))



