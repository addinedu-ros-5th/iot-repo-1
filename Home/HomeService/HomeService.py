import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic
from PyQt5.QtCore import *
import serial
import time
import urllib.request
import pyqtgraph as pg
import numpy as np
import cv2, imutils
import socket
import os
import threading
import datetime
import mysql.connector
# import pandas as pd

from_class = uic.loadUiType("HomeService.ui")[0]


class ImageClient:
    def __init__(self):
        self.host = '192.168.0.156'
        self.port = 9001
        

        
    def send_image_thread(self, image_path):
        self.thread = threading.Thread(target=self.send_image, args=(image_path,))
        self.thread.start()
    def send_image(self, image_path):
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((self.host, self.port))
            
            
            image_size = os.path.getsize(image_path)
            size_data = image_size.to_bytes(4, byteorder='big')
            client_socket.sendall(size_data)
            
            
            with open(image_path, 'rb') as image_file:
                while True:
                    image_data = image_file.read(1024)
                    if not image_data:
                        break
                    client_socket.sendall(image_data)
            print("이미지 데이터를 서버로 전송했습니다.")
            client_socket.close()
        except Exception as e:
            print(f"오류 발생: {e}")


class Camera(QThread):
    update = pyqtSignal()
    
    def __init__(self, sec=0, parent=None):
        super().__init__()
        self.main = parent
        self.running = True
        
    def run(self):
        count = 0
        while self.running == True:
            self.update.emit()
            time.sleep(0.1)
            
    def stop(self):
        self.running = False
        
class WindowClass(QMainWindow, from_class) :
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        
        
        
        
        self.arduino = serial.Serial('/dev/ttyACM0', 9600)
        self.danger_list = []
        # Camera Update
        self.serial_port = serial.Serial("/dev/ttyACM1", 9600, timeout=1)
        self.now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.camera_thread = Camera(self)
        self.camera_thread.daemon = True
        self.camera_thread.update.connect(self.update_camera)
        self.pixmap = QPixmap()

        self.btncard.clicked.connect(self.toggleMode)
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
        
        self.warning_shown = False
        
        self.update_timer = QTimer(self)  
        self.update_timer.timeout.connect(self.update_data)
        self.update_timer.start(100)
        
        self.db_connection = mysql.connector.connect(
            host="iot.cf64s86023q3.ap-northeast-2.rds.amazonaws.com",
            user="iot",
            password="1234",
            database="iot"
        )
        self.cursor = self.db_connection.cursor()
        if self.db_connection.is_connected():
            print("MySQL connected successfully!")
        else:
            print("MySQL connection failed.")

    def toggleMode(self):
        # 현재 버튼의 텍스트 가져오기
        current_text = self.btncard.text()

        # 버튼의 텍스트에 따라 모드 변경
        if current_text == "Registration":
            new_text = "Verification"
            mode = 'v'
        else:
            new_text = "Registration"
            mode = 'r'

        # 버튼의 텍스트 변경
        self.btncard.setText(new_text)

        # 모드에 따라 시리얼 통신
        self.sendSerial(mode)

    def sendSerial(self, mode):
        # 시리얼 통신으로 모드 전송
        self.serial_port.write(mode.encode())
        print(f"Sent mode: {mode}")

    def update_camera(self):
        # if self.video is not None:
        retval, image = self.video.read()
        if retval:
            self.image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            h, w, c = self.image.shape
            qimage = QImage(self.image.data, w, h, w*c, QImage.Format_RGB888)
            
            self.pixmap = self.pixmap.fromImage(qimage)
            self.pixmap = self.pixmap.scaled(self.lbcamera.width(), self.lbcamera.height())
            
            self.lbcamera.setPixmap(self.pixmap)
            
    def capture_image(self):
    # 카메라에서 이미지 가져오기
        ret, frame = self.video.read()
        if ret:
            # 이미지를 파일로 저장할 경로 설정
            self.now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            # 이미지 저장
            cv2.imwrite("/home/hb/Desktop/image/" + self.now + '.jpg', frame)
            # 저장된 이미지 경로 출력
            print("Save Success")
            
            self.stop_recording()
            self.lbcamera.clear()
            self.image = ImageClient()
            self.image.send_image_thread('/home/hb/Desktop/image/' + self.now + '.jpg')
            
            query = "INSERT INTO danger_data VALUES (%s, %s)"
            value = (self.now, 3)
            self.cursor.execute(query, value)
            self.db_connection.commit()
            
                
    # def clickCamera(self):
    #     self.timer = QTimer(self)
    #     self.timer.timeout.connect(self.start_recording)
    #     self.timer.start(5000)
    #     # self.start_recording()
        
    def stop_recording(self):
        
        self.camera_thread.running = False
        self.count = 0
        self.camera_thread.stop()
        self.video.release()
        #self.lbcamera.clear()
        
    def start_recording(self):
        # self.camera_thread.stop()
        self.camera_thread.running = True
        self.camera_thread.start()
        self.video = cv2.VideoCapture(0)
        
        QTimer.singleShot(3000, self.capture_image)
        self.danger_list.append(3)
        #self.image_client.start()
        
    def show_warning_message(self):
        # if not self.warning_shown:
        self.warning_message_box = QMessageBox(self)
        self.warning_message_box.setWindowTitle("Warning!")
        self.warning_message_box.setText("Dangerous Level : 2")
        self.warning_message_box.setStandardButtons(QMessageBox.Ok)
        self.warning_message_box.exec_()
        
        self.warning_shown = True
        
        # Start a timer for 5 seconds
        self.response_timer = QTimer()
        self.response_timer.setSingleShot(True)
        self.response_timer.timeout.connect(self.check_response)
        self.response_timer.start(5000)
        
        
    def check_response(self):
        if self.warning_message_box.isVisible():
            self.start_recording()
            
            self.warning_message_box.close()
        else:
            print("User responded")


    
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
            # self.danger_list.append(self.sensor_data[6:])
            # print(self.danger_list)
            danger_status = int(self.sensor_data[16])
        
            
            if sensor_value:
                if len(self.sensor_data) == 17:
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
                    self.lbdanger.setText("위험 단계 : " + str(danger_status))
                    self.lbdanger.setAlignment(Qt.AlignCenter)
                    self.setStyleSheet("font-size: 20px;")



                if danger_status == 2:
                    query = "INSERT INTO danger_data VALUES (%s, %s)"
                    value = (self.now, danger_status)
                    self.cursor.execute(query, value)
                    self.db_connection.commit()
                    # self.danger_list.append(danger_status)
                    self.show_warning_message()
                    
                
                # Gragh Reset
                
                if len(self.dust_x_data) == 50:
                    
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
    # client = ImageClient()
    # client.send_image_thread('captured_image.jpg')100
