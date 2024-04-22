import sys
import socket
import json
from PyQt5.QtCore import QTimer, QUrl
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5 import uic
from PyQt5.QtGui import QDesktopServices
import serial
import time
from selenium import webdriver
from selenium.webdriver.common.by import By
import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic
from PyQt5.QtCore import *
import threading
import socket

from_class = uic.loadUiType("care_pc1.ui")[0]

class WindowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.uids = [None, None]  # UID를 저장할 배열 초기화
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600)  # 아두이노 포트에 따라 변경

        self.btn_enter.clicked.connect(self.sendUID)
        self.btn_clear.clicked.connect(self.clearFields)  # 수정 버튼과 연결
        self.btn_call.clicked.connect(self.callEmergency)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.readData)
        self.timer.start(100)  # 100ms마다 데이터 수신 확인
        self.pic = QLabel(self)
    
        self.start_image_server()
        
    def start_image_server(self):
        self.image_server = ImageServer()
        self.image_thread = threading.Thread(target=self.image_server.receive_image, args=(self.label,))
        self.image_thread.start()

    def readData(self):
        if self.serial_port.in_waiting > 0:
            data = self.serial_port.readline().decode().strip()
            if data:
                if self.uids[0] is None:
                    self.uids[0] = data
                else:
                    self.uids[1] = data
                self.updateUI()

    def updateUI(self):
        self.text_edit.clear()
        for i, uid in enumerate(self.uids):
            if uid is not None:
                self.text_edit.append(f"{uid}")

    def sendUID(self):
        # 두 번째 피시로 데이터 전송
        HOST = '192.168.0.67'  # 두 번째 피시의 IP 주소
        PORT = 12345           # 포트 번호

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.connect((HOST, PORT))
                
                # 모든 UID를 전송
                packed_data = json.dumps(self.uids).encode()
                s.sendall(packed_data)
                print("UID data sent successfully!")
                
                # 카드 정보를 입력 비활성화
                self.text_edit.setReadOnly(True)
                self.nameEdit.setReadOnly(False)
                self.phoneEdit.setReadOnly(False)
                self.addEdit.setReadOnly(False)
                
            except Exception as e:
                print("Failed to send UID data:", e)

    def clearFields(self):
        # 입력 필드 초기화 및 수정 가능하도록 변경

        self.uids[1] = None

        self.text_edit.clear()
        self.text_edit.setReadOnly(True)
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
class ImageServer:
    def __init__(self):
        self.host = '127.0.0.1'  
        self.port = 8000  

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

            pixmap = QPixmap()
            pixmap.loadFromData(bytes(image_data))
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
