import sys
import socket
import json
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from PyQt5.QtGui import QPixmap
import serial
import time
from selenium import webdriver
from selenium.webdriver.common.by import By
import threading
from PyQt5 import uic
from_class = uic.loadUiType("care_service.ui")[0]
class WindowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.uids = [None, None]  # UID를 저장할 배열 초기화
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600)  # 아두이노 포트에 따라 변경
        self.btn_enter.clicked.connect(self.sendUID)
        self.btn_clear.clicked.connect(self.clearFields)
        self.btn_call.clicked.connect(self.callEmergency)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.readData)
        self.timer.start(100)  # 100ms마다 데이터 수신 확인
        # 카드 정보 입력 필드를 읽기 전용으로 설정
        self.nameEdit.setReadOnly(True)
        self.phoneEdit.setReadOnly(True)
        self.addEdit.setReadOnly(True)
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
        HOST = '192.168.0.156'  # 두 번째 피시의 IP 주소
        PORT = 12345           # 포트 번호
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.connect((HOST, PORT))
                # None 값을 1으로 변환
                converted_uids = [uid if uid is not None else '00 00 00 00' for uid in self.uids]
                # 모든 UID를 전송
                packed_data = json.dumps(converted_uids).encode()
                s.sendall(packed_data)
                print("UID data sent successfully!")
                # 카드 정보를 입력 비활성화
                self.text_edit.setReadOnly(True)
                QMessageBox.information(self, "Success", "UID data sent successfully!")
            except Exception as e:
                print("Failed to send UID data:", e)
                QMessageBox.warning(self, "Error", f"Failed to send UID data: {e}")
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
if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()
    sys.exit(app.exec_())
