import socket
import struct
import serial
import json

def receive_data():
    HOST = '192.168.0.67'  # 두 번째 피시의 IP 주소
    PORT = 12345           # 포트 번호
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        conn, addr = s.accept()
        with conn:
            print('Connected by', addr)
            received_data = conn.recv(4096)  # 데이터 수신
            unpacked_data = json.loads(received_data.decode())
            print('Received data:', unpacked_data)
            if unpacked_data:
                compare_with_serial_data(unpacked_data)  # 시리얼 데이터와 비교
            else:
                print("No data received from the second PC")

def compare_with_serial_data(unpacked_data):
    ser = serial.Serial('/dev/ttyACM0', 9600)  # 아두이노 포트에 따라 변경
    while True:
        if ser.in_waiting > 0:
            serial_data = ser.readline().decode().strip()  # 시리얼 데이터 수신 및 디코딩
            print("Received from serial:", serial_data)  # 시리얼 데이터 출력
            if serial_data and unpacked_data:  # None이 아닌 경우에만 비교
                if unpacked_data[0] in serial_data:
                    # 첫 번째 UID 값과 일치하는 경우
                    ser.write(b'y')  # 아두이노로 'y'를 보냄
                    print("Servo rotated to 90[0] degrees")
                    break  # 루프 종료
                elif unpacked_data[1] in serial_data:
                    # 두 번째 UID 값과 일치하는 경우
                    ser.write(b'y')  # 아두이노로 'n'을 보냄
                    print("Servo rotated to 90[1] degrees")
                    break  # 루프 종료
                else:
                    # 두 UID 값 모두 일치하지 않는 경우
                    ser.write(b'n')  # 아두이노로 'n'을 보냄
                    print("Neither UID matched, servo not rotated")
                    break  # 루프 종료
            else:
                print("Serial data or unpacked data is None, skipping comparison")


if __name__ == '__main__':
    receive_data()
