import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic
from PyQt5.QtCore import *
import serial
import urllib.request
import pyqtgraph as pg
import numpy as np


from_class = uic.loadUiType("one_person.ui")[0]

class Camera(QThread):
    update = pyqtSignal()
    
    def __init__(self, sec=0, parent=None):
        super().__init__()
        self.main = parent
        self.running = True
        
    def run(self):
        count = 0
        while self.running:
            self.update.emit()
            time.sleep(0.1)
            
    def stop(self):
        self.running = False
        
class WindowClass(QMainWindow, from_class) :
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        
        self.arduino = serial.Serial('/dev/ttyACM0', 9600)
        
        
        # Dust 그래프 
        self.dust_x_data = []
        self.dust_y_data = []
        
        self.dust_graphWidget = pg.PlotWidget(self)
        self.dust_graphWidget.setGeometry(0, 40, 300, 180)
        self.dust_graphWidget.setBackground('w')  
        self.dust_graphWidget.showGrid(x=False, y=False)
        
        y200 = pg.InfiniteLine(pos=200, angle=0, pen=pg.mkPen(color='g', width=1))
        y250 = pg.InfiniteLine(pos=250, angle=0, pen=pg.mkPen(color='b', width=1))
        y300 = pg.InfiniteLine(pos=300, angle=0, pen=pg.mkPen(color='r', width=1))
        self.dust_graphWidget.addItem(y200)
        self.dust_graphWidget.addItem(y250)
        self.dust_graphWidget.addItem(y300)
        
        # MQ7 그래프
        self.mq7_x_data = []
        self.mq7_y_data = []
        self.mq7_graphWidget = pg.PlotWidget(self)
        self.mq7_graphWidget.setGeometry(330, 40, 300, 180)
        self.mq7_graphWidget.setBackground('w')
        self.mq7_graphWidget.showGrid(x=False, y=False)
        y5 = pg.InfiniteLine(pos=0.5, angle=0, pen=pg.mkPen(color='g', width=1))
        y7 = pg.InfiniteLine(pos=0.7, angle=0, pen=pg.mkPen(color='b', width=1))
        y9 = pg.InfiniteLine(pos=0.9, angle=0, pen=pg.mkPen(color='r', width=1))
        self.mq7_graphWidget.addItem(y5)
        self.mq7_graphWidget.addItem(y7)
        self.mq7_graphWidget.addItem(y9)
        
        # MQ135 그래프
        self.mq135_x_data = []
        self.mq135_y_data = []
        self.mq135_graphWidget = pg.PlotWidget(self)
        self.mq135_graphWidget.setGeometry(0, 260, 300, 180)
        self.mq135_graphWidget.setBackground('w')
        self.mq135_graphWidget.showGrid(x=False, y=False)
        y52 = pg.InfiniteLine(pos=0.2, angle=0, pen=pg.mkPen(color='g', width=1))
        y72 = pg.InfiniteLine(pos=0.4, angle=0, pen=pg.mkPen(color='b', width=1))
        y92 = pg.InfiniteLine(pos=0.6, angle=0, pen=pg.mkPen(color='r', width=1))
        self.mq135_graphWidget.addItem(y52)
        self.mq135_graphWidget.addItem(y72)
        self.mq135_graphWidget.addItem(y92)
        
        # MQ2 그래프
        self.mq2_x_data = []
        self.mq2_y_data = []
        self.mq2_graphWidget = pg.PlotWidget(self)
        self.mq2_graphWidget.setGeometry(330, 260, 300, 180)
        self.mq2_graphWidget.setBackground('w')
        self.mq2_graphWidget.showGrid(x=False, y=False)
        y2 = pg.InfiniteLine(pos=2, angle=0, pen=pg.mkPen(color='g', width=1))
        y4 = pg.InfiniteLine(pos=4, angle=0, pen=pg.mkPen(color='b', width=1))
        y6 = pg.InfiniteLine(pos=6, angle=0, pen=pg.mkPen(color='r', width=1))
        self.mq2_graphWidget.addItem(y2)
        self.mq2_graphWidget.addItem(y4)
        self.mq2_graphWidget.addItem(y6)
        
        # resistance_value 그래프
        self.resistance_x_data = []
        self.resistance_y_data = []
        self.resistance_graphWidget = pg.PlotWidget(self)
        self.resistance_graphWidget.setGeometry(0, 500, 300, 180)
        self.resistance_graphWidget.setBackground('w')
        self.resistance_graphWidget.showGrid(x=False, y=False)
        y20 = pg.InfiniteLine(pos=20, angle=0, pen=pg.mkPen(color='g', width=1))
        y40 = pg.InfiniteLine(pos=40, angle=0, pen=pg.mkPen(color='b', width=1))
        y60 = pg.InfiniteLine(pos=60, angle=0, pen=pg.mkPen(color='r', width=1))
        self.resistance_graphWidget.addItem(y20)
        self.resistance_graphWidget.addItem(y40)
        self.resistance_graphWidget.addItem(y60)
        
        # Sound 그래프
        self.sound_x_data = []
        self.sound_y_data = []
        self.sound_graphWidget = pg.PlotWidget(self)
        self.sound_graphWidget.setGeometry(330, 500, 300, 180)
        self.sound_graphWidget.setBackground('w')
        self.sound_graphWidget.showGrid(x=False, y=False)
        y250 = pg.InfiniteLine(pos=250, angle=0, pen=pg.mkPen(color='g', width=1))
        y500 = pg.InfiniteLine(pos=500, angle=0, pen=pg.mkPen(color='b', width=1))
        y750 = pg.InfiniteLine(pos=750, angle=0, pen=pg.mkPen(color='r', width=1))
        self.sound_graphWidget.addItem(y250)
        self.sound_graphWidget.addItem(y500)
        self.sound_graphWidget.addItem(y750)
        
        
        
        
        self.update_timer = QTimer(self)  
        self.update_timer.timeout.connect(self.update_data)
        self.update_timer.start(300)
        
    def update_data(self):
        if self.arduino.in_waiting:
            sensor_value = self.arduino.readline().decode().strip()
            values = list(map(float, sensor_value.split(',')))
            self.sensor_data = values
            
            dustdensity = self.sensor_data[0]
            mq7 = self.sensor_data[1]
            mq135 = self.sensor_data[2]
            mq2 = self.sensor_data[3]
            resistance_value = self.sensor_data[4]
            sound_value = self.sensor_data[5]
            
            if sensor_value:
                if len(self.sensor_data) == 6:
                    self.dust_x_data.append(len(self.dust_x_data))
                    self.dust_y_data.append(float(dustdensity))
                    self.mq7_x_data.append(len(self.mq7_x_data))
                    self.mq7_y_data.append(float(mq7))
                    self.mq135_x_data.append(len(self.mq135_x_data))
                    self.mq135_y_data.append(float(mq135))
                    self.mq2_x_data.append(len(self.mq2_x_data))
                    self.mq2_y_data.append(float(mq2))
                    self.resistance_x_data.append(len(self.resistance_x_data))
                    self.resistance_y_data.append(float(resistance_value))
                    self.sound_x_data.append(len(self.sound_x_data))
                    self.sound_y_data.append(float(sound_value))
                    
                    
                    self.dust_graphWidget.plot(self.dust_x_data, self.dust_y_data, pen=pg.mkPen(color='k', width=1))
                    self.dust_graphWidget.setYRange(0, 400)
                    
                    self.mq7_graphWidget.plot(self.mq7_x_data, self.mq7_y_data, pen=pg.mkPen(color='k', width=1))
                    self.mq7_graphWidget.setYRange(0, 1)
                    
                    self.mq135_graphWidget.plot(self.mq135_x_data, self.mq135_y_data, pen=pg.mkPen(color='k', width=1))
                    self.mq135_graphWidget.setYRange(0, 1)
                    
                    self.mq2_graphWidget.plot(self.mq2_x_data, self.mq2_y_data, pen=pg.mkPen(color='k', width=1))
                    self.mq2_graphWidget.setYRange(0, 7)
                    
                    self.resistance_graphWidget.plot(self.resistance_x_data, self.resistance_y_data, pen=pg.mkPen(color='k', width=1))
                    self.resistance_graphWidget.setYRange(0, 70)
                    
                    self.sound_graphWidget.plot(self.sound_x_data, self.sound_y_data, pen=pg.mkPen(color='k', width=1))
                    self.sound_graphWidget.setYRange(0, 900)
                    
                    self.lbdust.setText('미세 먼지 : ' + str(dustdensity))
                    self.lbmq7.setText('MQ7 : ' + str(mq7))
                    self.lbmq135.setText('MQ135 : ' + str(mq135))
                    self.lbmq2.setText('MQ2 : ' + str(mq2))
                    self.lbresistance.setText("Resistance : " + str(resistance_value))
                    self.lbsound.setText("Sound : " + str(sound_value))
                    
                if sound_value >= 300:
                    QMessageBox.warning(self, "Sound Warning!", "위험 단계 - 1(Sound)")

                if dustdensity >= 300:
                    QMessageBox.warning(self, "Dustdensity Warning!", "위험 단계 - 1(Dustdensity")
                    
                if mq7 >= 0.9:
                    QMessageBox.warning(self, "MQ7 Warning!", "위험 단계 - 1(MQ7)")
                    
                if mq135 >= 0.6:
                    QMessageBox.warning(self, "MQ135 Warning!", "위험 단계 - 1(MQ135)")
                    
                if mq2 >= 6:
                    QMessageBox.warning(self, "MQ2 Warning!", "위험 단계 - 1(MQ2)")
                    
                if resistance_value >= 60:
                    QMessageBox.warning(self, "Resistance Warning!", "위험 단계 - 1(Resistance)")
                    
                
                # Gragh Reset
                
                if len(self.dust_x_data) == 20:
                    
                    self.dust_x_data.clear()
                    self.dust_y_data.clear()
                    self.dust_graphWidget.clear()
                    y200 = pg.InfiniteLine(pos=200, angle=0, pen=pg.mkPen(color='g', width=1))
                    y250 = pg.InfiniteLine(pos=250, angle=0, pen=pg.mkPen(color='b', width=1))
                    y300 = pg.InfiniteLine(pos=300, angle=0, pen=pg.mkPen(color='r', width=1))
                    self.dust_graphWidget.addItem(y200)
                    self.dust_graphWidget.addItem(y250)
                    self.dust_graphWidget.addItem(y300)
                    
                    
                    self.mq7_x_data.clear()
                    self.mq7_y_data.clear()
                    self.mq7_graphWidget.clear()
                    y5 = pg.InfiniteLine(pos=0.5, angle=0, pen=pg.mkPen(color='g', width=1))
                    y7 = pg.InfiniteLine(pos=0.7, angle=0, pen=pg.mkPen(color='b', width=1))
                    y9 = pg.InfiniteLine(pos=0.9, angle=0, pen=pg.mkPen(color='r', width=1))
                    self.mq7_graphWidget.addItem(y5)
                    self.mq7_graphWidget.addItem(y7)
                    self.mq7_graphWidget.addItem(y9)
                    
                    self.mq135_x_data.clear()
                    self.mq135_y_data.clear()
                    self.mq135_graphWidget.clear()
                    y52 = pg.InfiniteLine(pos=0.2, angle=0, pen=pg.mkPen(color='g', width=1))
                    y72 = pg.InfiniteLine(pos=0.4, angle=0, pen=pg.mkPen(color='b', width=1))
                    y92 = pg.InfiniteLine(pos=0.6, angle=0, pen=pg.mkPen(color='r', width=1))
                    self.mq135_graphWidget.addItem(y52)
                    self.mq135_graphWidget.addItem(y72)
                    self.mq135_graphWidget.addItem(y92)
                    
                    self.mq2_x_data.clear()
                    self.mq2_y_data.clear()
                    self.mq2_graphWidget.clear()
                    y2 = pg.InfiniteLine(pos=2, angle=0, pen=pg.mkPen(color='g', width=1))
                    y4 = pg.InfiniteLine(pos=4, angle=0, pen=pg.mkPen(color='b', width=1))
                    y6 = pg.InfiniteLine(pos=6, angle=0, pen=pg.mkPen(color='r', width=1))
                    self.mq2_graphWidget.addItem(y2)
                    self.mq2_graphWidget.addItem(y4)
                    self.mq2_graphWidget.addItem(y6)
                    
                    self.resistance_x_data.clear()
                    self.resistance_y_data.clear()
                    self.resistance_graphWidget.clear()
                    y20 = pg.InfiniteLine(pos=20, angle=0, pen=pg.mkPen(color='g', width=1))
                    y40 = pg.InfiniteLine(pos=40, angle=0, pen=pg.mkPen(color='b', width=1))
                    y60 = pg.InfiniteLine(pos=60, angle=0, pen=pg.mkPen(color='r', width=1))
                    self.resistance_graphWidget.addItem(y20)
                    self.resistance_graphWidget.addItem(y40)
                    self.resistance_graphWidget.addItem(y60)
                    
                    self.sound_x_data.clear()
                    self.sound_y_data.clear()
                    self.sound_graphWidget.clear()
                    y250 = pg.InfiniteLine(pos=250, angle=0, pen=pg.mkPen(color='g', width=1))
                    y500 = pg.InfiniteLine(pos=500, angle=0, pen=pg.mkPen(color='b', width=1))
                    y750 = pg.InfiniteLine(pos=750, angle=0, pen=pg.mkPen(color='r', width=1))
                    self.sound_graphWidget.addItem(y250)
                    self.sound_graphWidget.addItem(y500)
                    self.sound_graphWidget.addItem(y750)
                    

if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()
    sys.exit(app.exec_())