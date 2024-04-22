import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic
from PyQt5.QtCore import *
import threading
import socket

from_class = uic.loadUiType("/home/ys/dev_ws/openCV/source/care_service/care_service.ui")[0]

        
class WindowClass(QMainWindow, from_class) :
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.pic = QLabel(self)
    
        self.start_image_server()
        
    def start_image_server(self):
        self.image_server = ImageServer()
        self.image_thread = threading.Thread(target=self.image_server.receive_image, args=(self.label,))
        self.image_thread.start()
        

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

# 클라이언트 pc에서 이미지 전송 코드
# import socket
# import os
# class ImageClient:
#     def __init__(self):
#         self.host = '127.0.0.1'  
#         self.port = 8000 
#     def send_image(self, image_path):
#         try:
#             client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             client_socket.connect((self.host, self.port))
            
#             image_size = os.path.getsize(image_path)
            
#             size_data = image_size.to_bytes(4, byteorder='big')
#             client_socket.sendall(size_data)
            
#             with open(image_path, 'rb') as image_file:
#                 while True:
#                     image_data = image_file.read(1024)
#                     if not image_data:
#                         break
#                     client_socket.sendall(image_data)
#             print("이미지 데이터를 서버로 전송했습니다.")
#             client_socket.close()
#         except Exception as e:
#             print(f"오류 발생: {e}")
# if __name__ == "__main__":
#     client = ImageClient()
#     client.send_image('/home/ys/dev_ws/20240415_132511.png')