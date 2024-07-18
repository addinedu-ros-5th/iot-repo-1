import rclpy as rp
from rclpy.node import Node
from ros_package_msgs.srv import CommandString
import threading

class RobotService(Node):
    def __init__(self):
        super().__init__('Robot_Service')

        # logistic_Manager로 robot_command 서비스를 받아옴
        self.robot_command_server = self.create_service(CommandString, '/robot_command', self.robot_command_callback)

        # Robot_control로 robot_command 서비스를 보냄
        self.robot_command_client = self.create_client(CommandString, 'Robot_control/robot_command')


    def robot_command_callback(self, request, response):
        self.get_logger().info(f'Received command: {request.command}')

        # 명령과 설명을 인자로 하여 스레드를 시작
        thread = threading.Thread(target=self.working_order, args=(request.command, request.description))
        thread.start()

        response.success = True
        response.message = f'Command {request.command} received and being processed'

        return response
    

    def working_order(self, command, description):
        if command == "human_detect":
            self.get_logger().info('Processing human_detect')
            self.send_robot_command(command, description)  # Robot_control: 작품설명 위치이동
        
        elif command == "description":
            self.get_logger().info('Processing description')
            self.send_robot_command(command, description)  # Robot_control: 제자리 위치

        elif command == "guide":
            self.get_logger().info('Processing guide')
            self.send_robot_command(command, description)  # Robot_control: 작품으로 길안내

        elif command == "again":
            self.get_logger().info('Processing comeback')

        elif command == "comeback":
            self.get_logger().info('Processing comeback')
            self.send_robot_command(command, description)  # Robot_control: 작품설명 위치이동

        else:
            self.get_logger().error('Unknown command received')
        

    def send_robot_command(self, command: str, description: str = ""):
        if not self.robot_command_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Robot_command service not available')
            return

        request = CommandString.Request()
        request.command = command
        request.description = description

        future = self.robot_command_client.call(request)

        response = future

        if response.success:
            self.get_logger().info('Robot_command executed successfully (Robot to Driving): %s' % response.message)
        else:
            self.get_logger().error('Failed to execute User_command (Robot to Driving) : %s' % response.message)


def main(args=None):
    rp.init(args=args)

    robot_service = RobotService()

    rp.spin(robot_service)

    robot_service.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
