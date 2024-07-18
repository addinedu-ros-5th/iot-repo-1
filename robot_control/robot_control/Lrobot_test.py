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

# 로봇의 번호, 네비게이션 시간 정의, 상태 정의

ROBOT_NUMBER = "1"
NAVIGATION_TIME_FACTOR = 13
MIN_NAV_TIME = 5.0

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
    ERROR = 9 # Error occured
    MAINTAINING = 10 # Maintaining robot

# 로봇 노드
class LrobotMotor(Node):
    def __init__(self):
        super().__init__("lrobot_motor")

    # 함수 초기화 전체
    # ----------------------------------
    
    # BasicNavigator 인스턴스를 생성하고, 네이베이션을 활성화할 때까지 대기한다.
        # 그리고 그 밑으로 여러 초기화 함수들을 호출해서 필요한 파라미터, 퍼블리셔, 클라이언트, 타이머 등 설정한다.
        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()
        
        self.initialize_parameters()
        self.initialize_timer()

        self.set_initial_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0) # 1-1 초기 위치 설정
    
    def initialize_parameters(self):
        self.qos_profile = QoSProfile( # 메세지 품질 정의(QoS)
            reliability=ReliabilityPolicy.RELIABLE, # 신뢰성, 메세지가 손실되지 않도록
            durability=DurabilityPolicy.VOLATILE, # 퍼블리셔가 사라지면 메세지가 유지되지 않음
            depth=10 # 버퍼링할 수 있는 깊이
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
            # RobotStatus.AT_HOME: RobotStatus.HOME
        }
        # 포인트라는 파라미터 선언
        self.declare_parameter("points", "") 

        # 포인트와 라벨 값을 주고
        self.points_str = self.get_parameter("points").get_parameter_value().string_value
        self.positions, self.labels = self.parse_points(self.points_str)

        # 그리드 7x8 나눠서 포지션, 포인트, 라벨로 나눔
        for row in range(8):
            for col in range(7):
                if self.positions[row][col] is not None:
                    self.get_logger().info(f"Point ({row},{col}): Position={self.positions[row][col]}, Label={self.labels[row][col]}")

        self.waypoint_points = self.get_indices_for_label("Waypoint")

        self.invalid_points = self.get_indices_for_label("Invalid point")
        self.robot_points = self.get_indices_for_label(f"Robot{int(ROBOT_NUMBER)}")

        self.initialize_variables()
        # self.log_processed_parameters()
    
    # 초기 좌표 셋팅을 현재 과년된거로 수정하는 것도 괜찮다
    # 초기 위치 설정 함수 - 위에 선언해둠 / 얘랑 밑에 initialize_variables 둘 중 하나쓰는게 맞다
    def set_initial_pose(self, x, y, z, qx, qy, qz, qw): 
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.nav.get_clock().now().to_msg()
        initial_pose.pose.position.x = x
        initial_pose.pose.position.y = y
        initial_pose.pose.position.z = z
        initial_pose.pose.orientation.x = qx
        initial_pose.pose.orientation.y = qy
        initial_pose.pose.orientation.z = qz
        initial_pose.pose.orientation.w = qw
        self.nav.setInitialPose(initial_pose)
        
    # 로봇의 활동 상태, 현재 상태, 이동 상태, 위치 등을 초기화
    def initialize_variables(self):
        self.is_active = False
        self.store_id = ""
        self.RACK_id = ""
        self.dropoff_id = ""
        self.status = RobotStatus.HOME
        self.is_moving = 0

        self.store_points = []
        self.RACK_points = []
        self.dropoff_points = []

        self.current_point = self.robot_points[0] #tuple
        self.next_point = self.robot_points[0] #tuple
        self.current_position = self.get_position(self.robot_points[0]) #namedtuple
        self.next_position = self.get_position(self.robot_points[0]) #namedtuple
        # print(type(self.current_point))
        # print(type(self.current_position))
        self.current_yaw = 0.0
        self.diff_dist = 0.0
        self.attempt_count = 0
        self.max_attempts = 3

        self.x_shift = 0.0
        self.y_shift = 0.0
    
    # 타이머 초기화
    def initialize_timer(self):
        self.moving_timer = self.create_timer(1.0, self.moving_timer_callback)
    
    # 주행 관련 코드
    # ----------------------------------
    
    # 움직이고 있을 때
    def moving_timer_callback(self):
        if self.is_moving == 1:
            self.get_logger().info(f"move to {self.next_position}")
            self.get_logger().info(f"current point : {self.current_point}, next point : {self.next_point}")
            self.is_moving = 0
            self.send_goal(self.next_position)

    # 목표 위치로 로봇을 이동시키는 함수
    def send_goal(self, goal):
        x = goal.x
        y = goal.y
        goal_pose = self.get_goal_pose(x, y)
        
        distance = self.calc_diff(self.next_position, self.current_position)
        nav_time = max(distance * NAVIGATION_TIME_FACTOR, MIN_NAV_TIME)
    
        self.nav.goToPose(goal_pose)
        i = 0
        while not self.nav.isTaskComplete():
            # i = i + 1
            feedback = self.nav.getFeedback()
            if feedback and i % 5 == 0:
                # print("Distance remaining: " + "{:.2f}".format(feedback.distance_remaining) + " meters.")
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
            
    # 이동이 성공했는지 확인
    def check_succeed(self, current_position):
        diff_dist = self.calc_diff(self.next_position, current_position)
        if diff_dist <= 0.5:
            self.get_logger().info("Moving succeeded!")
            self.verify_checkpoint()
            self.attempt_count = 0
            return True
        else:
            return self.resend_goal()
        
    # 이동 실패 시, 목표 위치로 다시 이동을 시도
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
    
    # 목표 위치에 돡했는지 확인하고, 도착 시 필요한 요청을 보냄
    def verify_checkpoint(self):
        if self.next_point in self.store_points and self.status == RobotStatus.TO_STORE:
            self.update_status()
            self.request_module("ST")
            self.get_logger().info(f"R-{ROBOT_NUMBER} arrived at Store. So, status updated to {self.status.value}")
        elif self.next_point in self.RACK_points and self.status == RobotStatus.TO_RACK:
            self.update_status()
            self.request_module("KS")
            self.get_logger().info(f"R-{ROBOT_NUMBER} arrived at RACK. So, status updated to {self.status.value}")
        elif self.next_point in self.robot_points and self.status == RobotStatus.RETURNING:
            self.update_status()
            self.request_module("HM")
            self.get_logger().info(f"R-{ROBOT_NUMBER} arrived at Home. So, status updated to {self.status.value}")
        else:
            self.get_logger().info("On waypoints")
    
    # 목표 위치를 설정, 목표 포즈를 반환
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
    
    # def set_goal_pose(self, x, y, z, qx, qy, qz, qw): # 2. 목표 위치 설정 함수
    #     goal_pose = PoseStamped()
    #     goal_pose.header.frame_id = 'map'
    #     goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
    #     goal_pose.pose.position.x = x
    #     goal_pose.pose.position.y = y
    #     goal_pose.pose.position.z = z
    #     goal_pose.pose.orientation.x = qx
    #     goal_pose.pose.orientation.y = qy
    #     goal_pose.pose.orientation.z = qz
    #     goal_pose.pose.orientation.w = qw
    #     return goal_pose
    
    

    # 주요 콜백 및 유틸리티 함수
    # 주어진 포인트의 위치를 반환
    def get_position(self, point):
        row, col = point
        return self.positions[row][col]

    # 주어진 포인트의 라벨을 반환
    def get_label(self, point):
        row, col = point
        return self.labels[row][col]

    # 포인트 문자열을 파싱해서 포인트의 위치와 라벨을 반환
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
    
    #특정 라벨을 가진 포인트의 인덱스를 반환
    def get_indices_for_label(self, label_prefix):
        indices = []
        for row in range(5):
            for col in range(9):
                if self.labels[row][col].startswith(label_prefix):
                    indices.append((row, col))
        return indices
    
    # 현재 포인트와 다음 포인트를 업데이트
    def update_positions(self, current_point, next_point):
        self.current_point = current_point
        self.next_point = next_point
        self.current_position = self.get_position(current_point)
        self.next_position = self.get_position(next_point)




def main(args=None):
    # ROS2 초기화
    rp.init(args=args)
    # 멀티 스레드 생성
    executor = MultiThreadedExecutor()

    # 노드 생성
    lrobot_motor = LrobotMotor()

    # 각 노드를 executor에 추가하고, excutor.spin()을 호출해서 이벤트 루프를 시작
    executor.add_node(lrobot_motor)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rp.ok():
            executor.shutdown()
            lrobot_motor.destroy_node()
            rp.shutdown()


if __name__ == "__main__":
    main()