from enum import Enum
import rclpy as rp
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Int16
from std_srvs.srv import Trigger
# from interface_package.srv import Module, LocationInfo, NodeNum
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import math
from time import sleep
from collections import namedtuple


MAX_ATTEMPTS = 3
NAVIGATION_TIME_FACTOR = 13
MIN_NAV_TIME = 5.0
ROBOT_NUMBER = "1"

Position = namedtuple('Position', ['x', 'y'])

class RobotStatus(Enum):
    HOME = 0  # waiting at Home
    TO_STORE = 1  # moving to Store
    AT_STORE = 2  # arrived at Store
    TO_RACK = 3  # moving to Rack
    AT_RACK = 4  # arrived at Rack
    TO_DROPOFF = 5  # moving to DROPOFF
    AT_DROPOFF = 6  # arrived at DROPOFF
    RETURNING = 7  # returning to Home
    AT_HOME = 8  # Arrived at Home
    CHARGING = 9 # Charging 
    ERROR = 10 # Error occured
    MAINTAINING = 11 # Maintaining robot

class OrderStatus(Enum):
    DELIVERY_YET = "DY"
    DELIVERY_READY = "DR"
    DELIVERY_START = "DS"
    DELIVERY_FINISH = "DF"


class LrobotMotor(Node):
    def __init__(self):
        super().__init__("lrobot_motor")
        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()

        self.initialize_parameters()
        self.initialize_subs_and_pubs()
        self.initialize_clients()
        self.initialize_timer()

        self.get_logger().info(f"R-{ROBOT_NUMBER} robot start")

    # QoS 설정, 상태 전환 정의, 파라미터 선언 및 초기화, 포인트 데이터 파싱 및 초기화 작업
    def initialize_parameters(self): 
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.status_change = {
            RobotStatus.HOME: RobotStatus.TO_STORE,
            RobotStatus.TO_STORE: RobotStatus.AT_STORE,
            RobotStatus.AT_STORE: RobotStatus.TO_RACK,
            RobotStatus.TO_RACK: RobotStatus.AT_RACK,
            RobotStatus.AT_RACK: RobotStatus.TO_DROPOFF,
            RobotStatus.TO_DROPOFF: RobotStatus.AT_DROPOFF,
            RobotStatus.AT_DROPOFF: RobotStatus.RETURNING,
            RobotStatus.RETURNING: RobotStatus.AT_HOME,
            RobotStatus.AT_HOME: RobotStatus.CHARGING,
            RobotStatus.CHARGING: RobotStatus.RETURNING,
        }
        self.declare_parameter("points", "")
        self.points_str = self.get_parameter("points").get_parameter_value().string_value
        self.positions, self.labels = self.parse_points(self.points_str)
        self.waypoint_points = self.get_indices_for_label("Waypoint")
        self.invalid_points = self.get_indices_for_label("Invalid point")
        self.robot_points = self.get_indices_for_label(f"Robot{int(ROBOT_NUMBER)}")
        self.initialize_variables()
        
    # 변수 초기화 작업
    def initialize_variables(self):
        self.is_active = False
        self.store_id = ""
        self.kiosk_id = ""
        self.status = RobotStatus.HOME
        self.is_moving = 0

        self.store_points = []
        self.kiosk_points = []

        self.current_point = self.robot_points[0]
        self.next_point = self.robot_points[0]
        self.current_position = self.get_position(self.robot_points[0])
        self.next_position = self.get_position(self.robot_points[0])
        self.current_yaw = 0.0
        self.diff_dist = 0.0
        self.attempt_count = 0
        self.max_attempts = 3

        self.x_shift = 0.0
        self.y_shift = 0.0

    # 이 부분이 토픽, 서비스 노드로 나눠서 사용 할 수 있을 뜻 그냥 별도의 노드를 만든다는 마인드
    # def initialize_subs_and_pubs(self):
    #     self.cmd_vel_pub = self.create_publisher(Twist, "/base_controller/cmd_vel_unstamped", 10)

    # def initialize_clients(self):
    #     self.reset_sub = self.create_service(Trigger, "/reset", self.reset_callback)
    #     self.robot_arrival_client = self.create_client(NodeNum, "robotArrival")
    #     self.module_client = self.create_client(Module, "module")

    # def initialize_timer(self):
    #     self.moving_timer = self.create_timer(1.0, self.moving_timer_callback)

    # 포인트 위치와 라벨을 반환 - 웨이 포인트 좌표 찍어서 사용해서 그런듯
    def get_position(self, point):
        row, col = point
        return self.positions[row][col]

    def get_label(self, point):
        row, col = point
        return self.labels[row][col]

    # 포인트 문자열을 파싱해서 위치와 라벨을 설정
    def parse_points(self, points_str):
        points = points_str.strip().split(";")
        positions = [[None for _ in range(9)] for _ in range(5)]
        labels = [[None for _ in range(9)] for _ in range(5)]

        for point in points:
            if point:
                key, value = point.split(": ")
                row, col = map(int, key.split(","))
                label, x, y = value.split(", ")
                positions[row][col] = Position(float(x), float(y))
                labels[row][col] = label

        return positions, labels
    
    # 특정 라벨 프리픽스를 가진 포인트의 인덱스를 반환
    def get_indices_for_label(self, label_prefix):
        indices = []
        for row in range(5):
            for col in range(9):
                if self.labels[row][col].startswith(label_prefix):
                    indices.append((row, col))
        return indices
    
    # 현재 및 다음 포인트와 위치를 업데이트
    def update_positions(self, current_point, next_point):
        self.current_point = current_point
        self.next_point = next_point
        self.current_position = self.get_position(current_point)
        self.next_position = self.get_position(next_point)

    # 로봇의 상태를 업데이트하고 필요한 작업을 수행
    def update_status(self):
        self.get_logger().info(f"Updating status from {self.status.name}")
        
        self.status = self.status_change[self.status]
        self.get_logger().info(f"Status updated to {self.status.name}")

        if self.status == RobotStatus.RETURNING:
            self.returning()
            self.get_logger().info("Returning to Home")
        
        if self.status == RobotStatus.AT_HOME:
            self.reset()
            sleep(1)
            self.status = RobotStatus.HOME
            self.get_logger().info(f"Status updated to {self.status.name}")

    # 타이머 콜백으로, 로봇이 움직일 때 실행된다. 목표 위치로 이동을 시작함
    def moving_timer_callback(self):
        if self.is_moving == 1:
            self.get_logger().info(f"move to {self.next_position}")
            self.get_logger().info(f"current point : {self.current_point}, next point : {self.next_point}")
            self.is_moving = 0
            self.send_goal(self.next_position)
            
    # 목표 위치로 이동 명령을 보낸다. 네비게이션 결과에 따라 후속 작업을 수행
    def send_goal(self, goal):
        x = goal.x
        y = goal.y
        goal_pose = self.get_goal_pose(x, y)
        
        distance = self.calc_diff(self.next_position, self.current_position)
        nav_time = max(distance * NAVIGATION_TIME_FACTOR, MIN_NAV_TIME)
    
        self.nav.goToPose(goal_pose)
        i = 0
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            if feedback and i % 5 == 0:
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds= nav_time):
                    self.nav.cancelTask()
        result = self.nav.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
        elif result == TaskResult.CANCELED:
            self.get_logger().warn("Goal was canceled!")
        elif result == TaskResult.FAILED:
            self.get_logger().warn("Goal failed!")

        if self.check_succeed(self.current_position):
            self.request_robot_arrival(self.next_point)

    # 현재 위치와 다음 위치를 기반으로 로봇의 방향(yaw)을 결정
    def determine_direction(self, current, next):
        dx = next[0] - current[0]
        dy = next[1] - current[1]
        self.get_logger().info(f"Determine direction: current={current}, next={next}, dx={dx}, dy={dy}, status={self.status}")

        if dx > 0 and dy == 0:
            yaw = 0.0
            self.x_shift == 0.0
            self.y_shift == 0.0
        elif dx < 0 and dy == 0:
            yaw = math.pi
            self.x_shift == -0.1
            self.y_shift == 0.0
        elif dx == 0 and dy > 0:
            yaw = math.pi / 2
            self.x_shift == 0.0
            self.y_shift == -0.05
        elif dx == 0 and dy < 0:
            yaw = -math.pi / 2
            self.x_shift == 0.0
            self.y_shift == 0.05
        else:
            yaw = math.atan2(dy, dx)

        if next in self.store_points:
            yaw = math.pi
        elif next in self.kiosk_points:
            if self.store_id[2] == "1":
                yaw = -math.pi / 2
            elif self.store_id == "2":
                yaw = math.pi / 2
        elif next in self.robot_points:
            yaw = 0.0

        return yaw

    # 목표 위치의 포즈 생성
    def get_goal_pose(self, x, y):
        yaw = self.determine_direction(self.current_point, self.next_point)
        self.get_logger().info(f"goal yaw : {yaw}")

        tmp = [0, 0, yaw]
        orientation_val = quaternion_from_euler(tmp[0], tmp[1], tmp[2])
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = x + self.x_shift
        goal_pose.pose.position.y = y + self.y_shift
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = orientation_val[2]
        goal_pose.pose.orientation.w = orientation_val[3]

        return goal_pose

    # 현재 위치와 목표 위치 간의 거리를 계산
    def calc_diff(self, goal_position, current_position):
        self.get_logger().info(f"current_position value: ({self.current_position.x}, {self.current_position.y})")
        self.get_logger().info(f"goal position: ({goal_position.x}, {goal_position.y})")

        diff_x = goal_position.x - current_position.x
        diff_y = goal_position.y - current_position.y
        diff = math.sqrt(diff_x ** 2 + diff_y ** 2)
        self.get_logger().info(f"diff : {diff}")
        return diff

    # 목표 위치 도달 여부를 확인
    def check_succeed(self, current_position):
        diff_dist = self.calc_diff(self.next_position, current_position)
        if diff_dist <= 0.5:
            self.get_logger().info("Moving succeeded!")
            self.verify_checkpoint()
            self.attempt_count = 0
            return True
        else:
            return self.resend_goal()
    # 목표 위치 미도달 시 목표 재송신
    def resend_goal(self):
        if self.attempt_count < 3:
            self.attempt_count += 1
            self.send_goal(self.next_position)
            self.get_logger().warn(f"Moving failed! Attempt {self.attempt_count}/3")
            return False
        else:
            self.get_logger().error("Moving failed after 3 attempts.")
            self.attempt_count = 0
            self.verify_checkpoint()
            return True
    # 현재 위치에 따른 체크포인트를 확인하고 상태를 업데이트
    def verify_checkpoint(self):
        if self.next_point in self.store_points and self.status == RobotStatus.TO_STORE:
            self.update_status()
            self.request_module("ST")
            self.get_logger().info(f"R-{ROBOT_NUMBER} arrived at Store. So, status updated to {self.status.value}")
        elif self.next_point in self.kiosk_points and self.status == RobotStatus.TO_KIOSK:
            self.update_status()
            self.request_module("KS")
            self.get_logger().info(f"R-{ROBOT_NUMBER} arrived at Kiosk. So, status updated to {self.status.value}")
        elif self.next_point in self.robot_points and self.status == RobotStatus.RETURNING:
            self.update_status()
            self.request_module("HM")
            self.get_logger().info(f"R-{ROBOT_NUMBER} arrived at Home. So, status updated to {self.status.value}")
        else:
            self.get_logger().info("On waypoints")

    # 모듈 서비스 요청 및 응답 처리
    # def request_module(self, location):
    #     self.get_logger().info(f"Request status : {location}")
    #     module_request = Module.Request()
    #     module_request.data = location
    #     future = self.module_client.call_async(module_request)
    #     future.add_done_callback(self.response_module)

    # def response_module(self, future):
    #     try:
    #         response = future.result()
    #         self.get_logger().info(f"module response : {response.success}")
    #     except Exception as e:
    #         self.get_logger().error(f"module call failed {e}")

    # 로봇 도착 서비스 요청 및 응답 처리
    # def request_robot_arrival(self, current_point):
    #     robot_arrival_request = NodeNum.Request()
    #     self.get_logger().info(f"Arrived at {current_point}")
    #     robot_arrival_request.current_x = current_point[0]
    #     robot_arrival_request.current_y = current_point[1]
    #     robot_arrival_request.next_x = 0
    #     robot_arrival_request.next_y = 0
    #     future = self.robot_arrival_client.call_async(robot_arrival_request)
    #     future.add_done_callback(self.response_robot_arrival)

    # def response_robot_arrival(self, future):
    #     try:
    #         response = future.result()
    #         self.get_logger().info(f"response {response.success} from TM")
    #     except Exception as e:
    #         self.get_logger().error(f"arrival call failed : {e}")


    # 리셋 서비스 요청 및 초기화 작업
    def reset_callback(self, request, response):
        self.reset()
        response.success = True
        response.message = "reset"
        return response
    
    def reset(self):
        self.get_logger().info("Reset")
        self.initialize_variables()

    # 로봇이 복귀할 떄의 상태를 초기화
    def returning(self):
        self.get_logger().info("Robot returning")
        self.store_points = []
        self.kiosk_points = []
        self.store_id = ""
        self.kiosk_id = ""
        self.is_active = False

# 로봇 상태를 퍼블리시
class DrobotStatus(Node):
    def __init__(self, motor_node):
        super().__init__("drobot_status")
        self.motor_node = motor_node
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.timer_period = 1.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.status_pub = self.create_publisher(Int16, "/status", self.qos_profile)
        
    # 로봇의 상태를 퍼블리시하는 메서드/ 
    def status_publish(self, status):
        msg = Int16()
        msg.data = status.value # RobotStatus Enum의 값을 가져와서 메세지 데이터로 설정
        self.status_pub.publish(msg) # 메세지 퍼블리시 
        # self.get_logger().info(f"Published status: {msg.data}")
    
    # 타이머가 만료 될때마다 호출되는 메서드    
    def timer_callback(self):
        self.status_publish(self.motor_node.status) # 인스턴스 상태를 가져와서 퍼블리시

class AmclSub(Node):
    def __init__(self, motor_node):
        super().__init__("amcl_sub_node")
        self.motor_node = motor_node # LrobotMotor 인스턴스를 참조하여 위치와 자세 정보를 업데이트 하기 위해 사용
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.listener_callback, 10)
        # /amcl__pose 토픽을 구독, P~tamped 메세지를 수신하는 섭을 생성, 메세지를 수신할 때마다 리스너 콜백 호출

    # 토픽에서 메세지를 수신할 때 호출되는 함수
    def listener_callback(self, msg):
        position = msg.pose.pose.position # 로봇의 현재 위치 정보를 가져옴
        orientation = msg.pose.pose.orientation # 로봇의 현제 자세 정보를 가져옴
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w) # 쿼터니언을 오리러 각도로 변환하여 roll, pitch, yaw 값을 얻음
        euler = euler_from_quaternion(quaternion)
        roll, pitch, yaw = euler

        self.motor_node.current_yaw = yaw # 현재 로봇의 yaw(회전 각도)를 업데이트함
        x = position.x
        y = position.y
        self.motor_node.current_position = Position(x, y) # 로봇의 현재 위치를 Position (namedtuple)로 업데이트함

        # self.motor_node.get_logger().info(f"Updated from amcl_pose: current_position value: {self.motor_node.current_position}")

# DrobotStatus 클래스는 로봇의 상태를 정기적으로 퍼블리시하는 역할을 합니다. DrobotMotor 노드의 현재 상태를 가져와 /status 토픽으로 퍼블리시합니다.
# AmclSub 클래스는 /amcl_pose 토픽을 구독하여 로봇의 현재 위치와 자세를 업데이트합니다. 
# 수신한 메시지의 위치와 자세 정보를 DrobotMotor 인스턴스에 업데이트하여 로봇의 현재 상태를 반영합니다.
# 이 두 노드는 함께 작동하여 로봇의 현재 상태를 지속적으로 모니터링하고 퍼블리시하는 역할을 합니다.