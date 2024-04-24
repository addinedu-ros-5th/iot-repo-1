import sys
import socket
import json
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
import mysql.connector
import serial
import time
from selenium import webdriver
from selenium.webdriver.common.by import By
import threading
from PyQt5 import uic
import os

from_class = uic.loadUiType("CareService.ui")[0]
class WindowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("CareService")

        self.pic = QLabel(self)
        self.start_image_server()
        self.btnupdate.clicked.connect(self.update)
        self.btn_enter.clicked.connect(self.save)
        self.btn_clear.clicked.connect(self.clearFields)
        self.btn_call.clicked.connect(self.callEmergency)
        self.timer = QTimer(self)

        self.timer.timeout.connect(self.update)  # QTimer timeout 이벤트를 update 함수와 연결
        self.timer.start(1000)  # 1초마다 update 함수 호출

        self.db_connection = mysql.connector.connect(
            host="iot.cf64s86023q3.ap-northeast-2.rds.amazonaws.com",
            user="iot",
            password="1234",
            database="iot"
        )
        self.cursor = self.db_connection.cursor(buffered = True)

        if self.db_connection.is_connected():
            print("MySQL connected successfully!")
        else:
            print("MySQL connection failed.")


    def start_image_server(self):
        self.image_server = ImageServer()
        self.image_thread = threading.Thread(target=self.image_server.receive_image, args=(self.label,))
        self.image_thread.start()

    def update(self):
        query = "SELECT * FROM danger_data order by sensing_time desc"
        self.cursor.execute(query)
        self.db_connection.commit()  # 커밋 후 커서 리셋
        data = self.cursor.fetchone()  # Fetch the last row
            
        # Populate dangerLevel line edit with fetched data
        if data:
            sensing_time, danger_status = data
            self.dangerLevel.setText(f"시간 : {sensing_time}, 위험 단계: {danger_status}")
        else:
            self.dangerLevel.setText("No data available")

    def populate_danger_level(self):
        # Fetch last row from MySQL table
        query = "SELECT * FROM danger_data order by sensing_time desc"
        self.cursor.execute(query)
        data = self.cursor.fetchone()  # Fetch the last row
            
        # Populate dangerLevel line edit with fetched data
        if data:
            sensing_time, danger_status = data
            self.dangerLevel.setText(f"시간 : {sensing_time}, 위험 단계: {danger_status}")
        else:
            self.dangerLevel.setText("No data available")

    def save(self):
        name = self.nameEdit.text()
        phone = self.phoneEdit.text()
        address = self.addEdit.text()
        query = "INSERT INTO example VALUES (%s, %s, %s)"
        values = (name, phone, address)
        self.cursor.execute(query, values)
        self.db_connection.commit()

        self.nameEdit.setReadOnly(True)
        self.phoneEdit.setReadOnly(True)
        self.addEdit.setReadOnly(True)

    def clearFields(self):
        # 입력 필드 초기화 및 수정 가능하도록 변경
        self.nameEdit.clear()
        self.nameEdit.setReadOnly(False)
        self.phoneEdit.clear()
        self.phoneEdit.setReadOnly(False)
        self.addEdit.clear()
        self.addEdit.setReadOnly(False)

    def callEmergency(self):
        # 셀레니움을 사용하여 웹 브라우저를 열고 작업 수행
        driver = webdriver.Chrome()  # Chrome 드라이버 사용
        driver.get("https://www.119.go.kr/Center119/regist.do#")
        # 이름 입력
        name = self.nameEdit.text()
        name_field = driver.find_element(By.XPATH, '//*[@id="dsr_name"]')
        name_field.send_keys(name)
        time.sleep(1)
        # 전화번호 입력
        phone = self.phoneEdit.text()
        phone1_field = driver.find_element(By.XPATH, '//*[@id="call_tel1"]')
        phone1_field.send_keys(phone[:3])
        phone2_field = driver.find_element(By.XPATH, '//*[@id="call_tel2"]')
        phone2_field.send_keys(phone[3:7])
        phone3_field = driver.find_element(By.XPATH, '//*[@id="call_tel3"]')
        phone3_field.send_keys(phone[7:])
        time.sleep(1)
        # 체크박스 체크
        check = driver.find_element(By.XPATH, '//*[@id="agree01"]')
        check.click()
        time.sleep(1)
        # 주소 입력
        address = self.addEdit.text()
        address_field = driver.find_element(By.XPATH, '//*[@id="juso2"]')
        address_field.send_keys(address)
        time.sleep(1)
        # 버튼 클릭
        save_btn = driver.find_element(By.XPATH, '//*[@id="save_btn"]')
        save_btn.click()
        time.sleep(1)
        # 작업이 완료되면 웹 브라우저를 닫습니다.
        QTimer.singleShot(3000,  driver.quit)  # 3초 대기 후 브라우저 종료



    def timerEvent(self, event):
        self.populate_danger_level()

class ImageServer:
    def __init__(self):
        self.host = '192.168.0.198'
        self.port = 9001
        self.image_folder = '/home/john/Desktop/image/'
    def receive_image(self, label):
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.bind((self.host, self.port))
            server_socket.listen(1)
            print(f"서버가 {self.host}:{self.port}에서 실행 중입니다.")
            client_socket, client_address = server_socket.accept()
            print(f"{client_address}에서 연결됨")
            size_data = client_socket.recv(4)
            image_size = int.from_bytes(size_data, byteorder='big', signed=False)
            image_data = bytearray()
            while len(image_data) < image_size:
                chunk = client_socket.recv(image_size - len(image_data))
                if not chunk:
                    raise RuntimeError("소켓 연결이 끊겼습니다.")
                image_data.extend(chunk)
            image_path = os.path.join(self.image_folder, 'received_image.jpg')
            with open(image_path, 'wb') as f:
                f.write(image_data)
            print('이미지를 저장했습니다', image_path)
            pixmap = QPixmap()
            pixmap.loadFromData(bytes(image_data))
            pixmap = pixmap.scaled(label.width(), label.height())
            label.setPixmap(pixmap)
            if not pixmap.isNull():
                pixmap = pixmap.scaled(label.width(), label.height())
                label.setPixmap(pixmap)
            client_socket.close()
            server_socket.close()
        except Exception as e:
            print(f"오류 발생: {e}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()
    sys.exit(app.exec_())
