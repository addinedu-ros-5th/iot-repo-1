import sys
from PyQt5.QtWidgets import QApplication, QWidget, QTextEdit, QVBoxLayout, QPushButton
from PyQt5.QtCore import QTimer
import serial
import socket
import json

class SerialReceiver(QWidget):
    def __init__(self):
        super().__init__()

        self.uids = [None, None]  # UID를 저장할 배열 초기화

        self.initUI()

    def initUI(self):
        self.setGeometry(100, 100, 400, 300)
        self.setWindowTitle('Serial Receiver')

        layout = QVBoxLayout()

        self.text_edit = QTextEdit()
        layout.addWidget(self.text_edit)

        # 전송 버튼 추가
        self.send_button = QPushButton('Send UID to 2nd PC', self)
        layout.addWidget(self.send_button)
        self.send_button.clicked.connect(self.sendUID)

        self.setLayout(layout)

        self.serial_port = serial.Serial('/dev/ttyACM0', 9600)  # 아두이노 포트에 따라 변경

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.readData)
        self.timer.start(100)  # 100ms마다 데이터 수신 확인

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
        HOST = '172.30.1.43'  # 두 번째 피시의 IP 주소
        PORT = 12345           # 포트 번호

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.connect((HOST, PORT))
                
                # 모든 UID를 전송
                packed_data = json.dumps(self.uids).encode()
                s.sendall(packed_data)
                print("UID data sent successfully!")
                
                # 화면 클리어 및 첫 번째 UID 출력
                self.text_edit.clear()
                self.text_edit.append(self.uids[0])
                self.uids[1] = None
                
            except Exception as e:
                print("Failed to send UID data:", e)



if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SerialReceiver()
    window.show()
    sys.exit(app.exec_())
