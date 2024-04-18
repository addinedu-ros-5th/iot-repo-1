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
        
        
        # Total data
        self.total_sensor_data = [0, 0, 0, 0, 0, 0]
        
        # 그래프의 x, y 데이터를 저장할 리스트 생성
        self.x_data = []
        self.y_data = []
        self.total = []
        self.light_data = []
        self.temperature_data = []
        self.humidity_data = []
        self.gas_data = []
        self.dust_data = []
        self.flammable_data = []
        
        
        # blind Servo
        self.servo_angle = 0
        
        # blind Control
        self.btnBlindOn.clicked.connect(self.blind_open)
        self.btnBlindOff.clicked.connect(self.blind_close)

        
        self.update_timer = QTimer(self)  
        self.update_timer.timeout.connect(self.update_data)
        self.update_timer.start(300)
        
        
        # Card Mode
        self.auth_mode = True
        self.btnCard.clicked.connect(self.cardstate)
        
        # Temperature Fan Control
        self.temp_mode = True
        self.btnFancontrol.clicked.connect(self.temp_control)
        
    # 카드 모드 변환
    def cardstate(self):
    
        self.auth_mode = not self.auth_mode
        self.update_button_text()
        
    def update_button_text(self):
        if self.auth_mode:
            self.btncard.setText("카드 인증")
        else:
            self.btncard.setText("카드 등록")
    
    # 에어컨 변환
    def temp_control(self):
        self.temp_mode = not self.temp_mode
        self.update_button()
        
    def update_button(self):
        if self.temp_mode:
            self.btnFancontrol.setText("Fan On")
            
        else:
            self.btnFancontrol.setText("Fan Off")
        
        

    
    
    def update_data(self):
        
        
        if self.arduino.in_waiting:
            sensor_value = self.arduino.readline().decode().strip()
            values = list(map(int, sensor_value.split(',')))
            self.sensor_data = values
            self.total.append(self.sensor_data)
            
            if sensor_value:
                #data = sensor_value
                #print(sensor_value)
                
                if len(self.sensor_data) == 6:
                    #light, temperature, humidity, a, b, c = map(int, data)
                    
                    self.light_data.append(self.sensor_data[3])
                    self.temperature_data.append(self.sensor_data[4])
                    self.humidity_data.append(self.sensor_data[5])
                    self.dust_data.append(self.sensor_data[0])
                    self.flammable_data.append(self.sensor_data[1])
                    self.gas_data.append(self.sensor_data[2])
                    
                    self.x_data.append(len(self.x_data))
                    self.y_data.append(float(self.sensor_data[3]))
                    
                    self.graphWidget.plot(self.x_data, self.y_data, pen=pg.mkPen(color='k', width=1))
                    self.graphWidget.setYRange(0, 100)
                    
                    self.lineEdit.setText('현재 밝기 : ' + str(self.sensor_data[3]))
                    self.lcdNumber.display(float(self.sensor_data[4]))
                    self.progressBar.setValue(int(self.sensor_data[5]))
                    self.lbDust.setText(str(self.sensor_data[0]))
                    self.lbGas_2.setText(str(self.sensor_data[2]))
                    self.lbFlammable_2.setText(str(self.sensor_data[1]))
                    
            
            
            
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


            
if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()
    sys.exit(app.exec_())