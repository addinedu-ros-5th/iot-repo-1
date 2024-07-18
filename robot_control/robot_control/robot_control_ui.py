#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotControlUI(Node):
    def __init__(self):
        super().__init__('robot_control_ui')
        self.publisher_ = self.create_publisher(String, 'robot_command', 10)
        self.init_ui()

    def init_ui(self):
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle('Robot Control')

        self.layout = QVBoxLayout()

        self.label = QLabel('Robot Status: Ready')
        self.layout.addWidget(self.label)

        self.button = QPushButton('Send Command')
        self.button.clicked.connect(self.send_command)
        self.layout.addWidget(self.button)

        self.window.setLayout(self.layout)
        self.window.show()
        sys.exit(self.app.exec_())

    def send_command(self):
        msg = String()
        msg.data = 'Go'
        self.publisher_.publish(msg)
        self.label.setText('Robot Status: Command Sent')

def main(args=None):
    rclpy.init(args=args)
    ui_node = RobotControlUI()
    rclpy.spin(ui_node)
    ui_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
