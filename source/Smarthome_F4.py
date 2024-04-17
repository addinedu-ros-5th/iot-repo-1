import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic
from PyQt5.QtCore import QTimer
import serial
import numpy as np
import pyqtgraph as pg
from collections import deque

from_class = uic.loadUiType("home_widget.ui")[0]

class WindowClass(QMainWindow, from_class) :
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        
        self.setWindowTitle("Arduino Data Display")
        self.setGeometry(100, 100, 940, 850)
        
        self.arduino = serial.Serial('/dev/ttyACM0', 9600)
        
        self.graphWidget = pg.PlotWidget(self)
        self.graphWidget.setGeometry(0, 50, 340, 200)
        self.graphWidget.setBackground('w')  
        self.graphWidget.showGrid(x=False, y=False)
        
        y80 = pg.InfiniteLine(pos=80, angle=0, pen=pg.mkPen(color='g', width=1))
        y65 = pg.InfiniteLine(pos=65, angle=0, pen=pg.mkPen(color='b', width=1))
        y50 = pg.InfiniteLine(pos=50, angle=0, pen=pg.mkPen(color='r', width=1))
        self.graphWidget.addItem(y50)
        self.graphWidget.addItem(y65)
        self.graphWidget.addItem(y80)
        
        
        
        
        # 그래프의 x, y 데이터를 저장할 리스트 생성
        self.x_data = []
        self.y_data = []
        self.total = []
        self.light_data = []
        self.temperature_data = []
        self.humidity_data = []

        # blind Servo
        self.servo_angle = 10
        
        # blind Control
        self.btnBlindOn.clicked.connect(self.blind_open)
        self.btnBlindOff.clicked.connect(self.blind_close)


        # Button Condition
        self.btnAuto.setCheckable(True)
        self.btnManual.setCheckable(True)
        self.btnEconomy.setCheckable(True)
        
        # Home GUI Button 
        self.btnAuto.clicked.connect(self.Auto_btn)
        self.btnAutoOn.clicked.connect(self.Auto_btn)
        self.btnAutoOff.clicked.connect(self.Auto_btn)
        self.btnManual.clicked.connect(self.Manual_btn)
        self.btnManualOn.clicked.connect(self.Manual_btn)
        self.btnManualOff.clicked.connect(self.Manual_btn)
        self.btnEconomy.clicked.connect(self.Economy_btn)
        self.btnEconomyOn.clicked.connect(self.Economy_btn)
        self.btnEconomyOff.clicked.connect(self.Economy_btn)
        
        self.update_timer = QTimer(self)  
        self.update_timer.timeout.connect(self.update_data)
        self.update_timer.start(500)
        
        

    
    
    def update_data(self):
        
        
        if self.arduino.in_waiting:
            sensor_value = self.arduino.readline().decode().strip()
            self.total.append(sensor_value)
            
            
            
            if sensor_value:
                data = sensor_value.split(',')
                
                if len(data) == 3:
                    light, temperature, humidity = map(int, data)
                    
                    self.light_data.append(light)
                    self.temperature_data.append(temperature)
                    self.humidity_data.append(humidity)
                    
                    
                    self.x_data.append(len(self.x_data))
                    self.y_data.append(float(light))
                    
                    self.graphWidget.plot(self.x_data, self.y_data, pen=pg.mkPen(color='k', width=1))
                    self.graphWidget.setYRange(0, 100)
                    
                    self.lineEdit.setText('현재 밝기 : ' + str(light))
                    
            
            
            
                if len(self.x_data) == 30:
                    self.x_data.clear()
                    self.y_data.clear()
                    self.graphWidget.clear()
                    y80 = pg.InfiniteLine(pos=80, angle=0, pen=pg.mkPen(color='g', width=1))
                    y65 = pg.InfiniteLine(pos=65, angle=0, pen=pg.mkPen(color='b', width=1))
                    y50 = pg.InfiniteLine(pos=50, angle=0, pen=pg.mkPen(color='r', width=1))
                    self.graphWidget.addItem(y50)
                    self.graphWidget.addItem(y65)
                    self.graphWidget.addItem(y80)

        
        
    def blind_open(self):
        self.servo_angle -= 90
        self.servo_angle = max(0, self.servo_angle)
        self.arduino.write(str(self.servo_angle).encode())

    def blind_close(self):
        self.servo_angle += 90
        self.servo_angle = min(180, self.servo_angle)
        self.arduino.write(str(self.servo_angle).encode())

    def Auto_btn(self):
        if self.btnAuto.isChecked():
            self.btnAutoOn.setStyleSheet("color: rgb(255, 255, 255); background-color: blue; border: 2px solid rgb(255, 255, 255); border-radius: 20px;")
            self.btnAutoOff.setStyleSheet("color: rgb(255, 20, 20); background-color: white; border: 2px solid rgb(255, 20, 20); border-radius: 20px;")
            
        else:
            self.btnAutoOn.setStyleSheet("color: rgb(58, 134, 255); background-color: white; border: 2px solid rgb(58, 134, 255); border-radius: 20px;")
            self.btnAutoOff.setStyleSheet("	color: rgb(255, 255, 255); background-color: red; border: 2px solid rgb(255, 255, 255); border-radius: 20px;")



    def Manual_btn(self):
        if self.btnManual.isChecked():
            self.btnManualOn.setStyleSheet("color: rgb(255, 255, 255); background-color: blue; border: 2px solid rgb(255, 255, 255); border-radius: 20px;")
            self.btnManualOff.setStyleSheet("color: rgb(255, 20, 20); background-color: white; border: 2px solid rgb(255, 20, 20); border-radius: 20px;")

        else:
            self.btnManualOn.setStyleSheet("color: rgb(58, 134, 255); background-color: white; border: 2px solid rgb(58, 134, 255); border-radius: 20px;")
            self.btnManualOff.setStyleSheet("	color: rgb(255, 255, 255); background-color: red; border: 2px solid rgb(255, 255, 255); border-radius: 20px;")



    def Economy_btn(self):
        if self.btnEconomy.isChecked():
            self.btnEconomyOn.setStyleSheet("color: rgb(255, 255, 255); background-color: blue; border: 2px solid rgb(255, 255, 255); border-radius: 20px;")
            self.btnEconomyOff.setStyleSheet("color: rgb(255, 20, 20); background-color: white; border: 2px solid rgb(255, 20, 20); border-radius: 20px;")

        else:
            self.btnEconomyOn.setStyleSheet("color: rgb(58, 134, 255); background-color: white; border: 2px solid rgb(58, 134, 255); border-radius: 20px;")
            self.btnEconomyOff.setStyleSheet("	color: rgb(255, 255, 255); background-color: red; border: 2px solid rgb(255, 255, 255); border-radius: 20px;")
    
    
    
            
if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()
    sys.exit(app.exec_())