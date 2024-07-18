#!/usr/bin/env python3
import rclpy as rp
import os
import time
import math
from rclpy.node import Node
from copy import deepcopy
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from rclpy.executors import MultiThreadedExecutor

from nav2_simple_commander.robot_navigator import BasicNavigator
import numpy as np
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import TaskResult
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan


ID = os.getenv('ROS_DOMAIN_ID', 'Not set')

class My_Location(Node):
    def __init__(self, control):
        super().__init__("robot_loc")
        self.control = control

        # 로봇의 현재 위치를 구독하는 서브스크립션
        self.subcription_amclpose = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.current_pose,
            10
        )

        # 맵 데이터를 구독하는 서브스크립션
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            10
        )

        # 레이저 스캔 데이터를 구독하는 서브스크립션
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.map_data = None
        self.map_info = None

    # 로봇의 현재 위치와 방향 데이터를 갱신하는 콜백 함수
    def current_pose(self, data):
        self.control.my_pose[0] = data.pose.pose.position.x
        self.control.my_pose[1] = data.pose.pose.position.y

        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w
        
        roll, pitch, yaw = self.euler_from_quaternion(x, y, z, w)
        self.control.my_yaw = yaw

    # 맵 데이터를 받는 콜백 함수
    def map_callback(self, data):
        self.map_data = data.data
        self.map_info = data.info

    # 레이저 스캔 데이터를 받는 콜백 함수
    def scan_callback(self, data):
        if self.map_data is not None and self.map_info is not None:
            for angle, distance in enumerate(data.ranges):
                if distance < data.range_max:
                    # 맵 프레임에서 장애물의 위치를 계산
                    theta = data.angle_min + angle * data.angle_increment
                    obs_x = self.control.my_pose[0] + distance * math.cos(theta + self.control.my_yaw)
                    obs_y = self.control.my_pose[1] + distance * math.sin(theta + self.control.my_yaw)
                    self.update_map_with_obstacle(obs_x, obs_y)

    # 장애물을 맵에 업데이트하는 함수
    def update_map_with_obstacle(self, obs_x, obs_y):
        map_x = int((obs_x - self.map_info.origin.position.x) / self.map_info.resolution)
        map_y = int((obs_y - self.map_info.origin.position.y) / self.map_info.resolution)
        index = map_y * self.map_info.width + map_x
        if 0 <= index < len(self.map_data):
            self.map_data[index] = 100  # 해당 셀을 점유된 상태로 표시

    # 쿼터니언을 오일러 각도로 변환하는 함수
    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

# 위의 클래스는 현재 위치와 방향 확인 및 실시간 장애물 업데이트 하는 노드임 그 이상 그 이하로 수정하려고 하지 마
# 밑에는 실질적인 로봇을 제어하기 위한 노드, 

class RobotControl(Node): 
    def __init__(self): 
        super().__init__("robot_control")
        self.get_logger().info("start robot_control")
        self.nav = BasicNavigator()
        
        self.current_task_location = ""
        # self.tasking = False
        # self.next_out = False
        # self.marker_service_done = False
        
        # self.target_type = ""
        # self.task_status = ""
        # self.task_id = ""
        # self.item = ""
        self.quantity = 0
        self.my_pose = [0.0, 0.0, 0.0]
        self.my_yaw = 0
        
        # self.current_path_msg = CurrentPath()
        
        # self.server = self.create_service(
        #     AllocateTask, 
        #     f"/allocate_task_{ID}", 
        #     self.task_callback
        # )

        
        # self.task_completion_publisher = self.create_publisher(
        #     TaskCompletion,
        #     "/task_completion",
        #     10
        # )

        # self.robot_status_publisher = self.create_publisher(
        #     RobotStatus,
        #     "/robot_status",
        #     10
        # )

        # self.current_path_publisher = self.create_publisher(
        #     CurrentPath,
        #     "/current_path",
        #     10
        # )
        
        # self.arucomarker_client = self.create_client(
        #     ArucoCommand, "/aruco_control"
        # )

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # 맵의 좌표를 미리 지정하는 단계
        # pose x, y, z
        # 추후 json or yaml로 변경 필요
        self.POSE_DICT = { 
        "R_A1" : [1.89 - 0.45, 0.45, 0], "R_A2" : [1.89 - 0.45, 0.45, 0], "R_A3" : [1.89 - 0.45, 0.45, 0],
        "R_B1" : [1.89 - 0.45, 1.05, 0], "R_B2" : [1.89 - 0.45, 1.05, 0], "R_B3" : [1.89 - 0.45, 1.05, 0],
        "R_C1" : [1.89 - 0.45, 1.65, 0], "R_C2" : [1.89 - 0.45, 1.65, 0], "R_C3" : [1.89 - 0.45, 1.65, 0],
        "R_D1" : [1.89 - 1.05, 0.45, 0], "R_D2" : [1.89 - 1.05, 0.45, 0], "R_D3" : [1.89 - 1.05, 0.45, 0],
        "R_E1" : [1.89 - 1.05, 1.05, 0], "R_E2" : [1.89 - 1.05, 1.05, 0], "R_E3" : [1.89 - 1.05, 1.05, 0],
        "R_F1" : [1.89 - 1.05, 1.65, 0], "R_F2" : [1.89 - 1.05, 1.65, 0], "R_F3" : [1.89 - 1.05, 1.65, 0],
        "I1" : [1.89 - 1.95, 0.75, 0], "I2" : [1.89 - 1.95, 0.75, 0],
        "O1" : [1.89 - 1.95, 1.05, 0], "O2" : [1.89 - 1.95, 1.65, 0],
        "RH1" : [1.89 - 1.35, 0.15, 0], "RH2" : [1.89 - 1.05, 0.15, 0]
        }

        self.YAW_DICT = { 
        "R_A1" : 3.14, "R_A2" : 3.14, "R_A3" : 3.14,
        "R_B1" : 3.14, "R_B2" : 3.14, "R_B3" : 3.14,
        "R_C1" : 3.14, "R_C2" : 3.14, "R_C3" : 3.14,
        "R_D1" : 3.14, "R_D2" : 3.14, "R_D3" : 3.14,
        "R_E1" : 3.14, "R_E2" : 3.14, "R_E3" : 3.14,
        "R_F1" : 3.14, "R_F2" : 3.14, "R_F3" : 3.14,
        "I1" : 1.57, "I2" : 4.71,
        "O1" : 1.57, "O2" : 4.71,
        "RH1" : 1.57, "RH2" : 4.71
        }
        
        
        # self.send_robot_status_topics()
        self.get_logger().info("control is ready")


    # 경로 계획을 시작함
    def follow_path(self, pose_name, LED):
        try:
            self.get_logger().info("path_planning start")
            
            target_pose = self.POSE_DICT[pose_name]
            target_yaw = self.YAW_DICT[pose_name]
            self.target_type = "Stopover"
            self.real_time_stopover_planning(target_pose)
            
            self.get_logger().info(f"goto{pose_name}")
            self.target_type = "Main"
            self.move_pose(target_pose, target_yaw)

        # 서비스 제어 코드 기입하면 됨, 예를 들어 도착 시 LED ON/OFF, 적재 시 LED ON/OFF
        # if 뭐시기
        
        except Exception as e:
            self.get_logger().error(f"{e}")
            return False

    def move_pose(self, target_pose, yaw):
        q = self.euler_to_quaternion(yaw=yaw)

        self.get_logger().info(f"pose : {target_pose} yaw : {yaw}")
        goal_pose = PoseStamped()
        redeem_vector = self.redeem_pose(target_pose, 0.25)
        
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()

        goal_pose.pose.position.x = target_pose[0] + redeem_vector[0]
        goal_pose.pose.position.y = target_pose[1] + redeem_vector[1]
        goal_pose.pose.position.z = target_pose[2] + redeem_vector[2]

        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]
        
        self.nav.goToPose(goal_pose)

        self.nav_distance_feedback(yaw)
        
    # 오일러 각도를 쿼터니언으로 변환
    def euler_to_quaternion(self, yaw=0, pitch=0, roll=0):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]

    # 목표 위치를 보정하기 위해 방향 벡터를 계산, 방향 벡터의 크기로 단위 벡터 구함, 단위 벡터에 거리 값을 곱해서 보정 벡터를 계산하고 반환
    def redeem_pose(self, target_pose, distance):
        x = target_pose[0]
        y = target_pose[1]
        current_x = self.my_pose[0]
        current_y = self.my_pose[1]
        direction_vector = [x - current_x, y - current_y, 0.0]

        magnitude = math.sqrt(direction_vector[0]**2 + direction_vector[1]**2 + direction_vector[2]**2)

        if magnitude == 0:
            raise ValueError 
        
        unit_vector = [direction_vector[0] / magnitude, direction_vector[1] / magnitude, direction_vector[2] / magnitude]
        redeem_vector = [unit_vector[0] * distance, unit_vector[1] * distance, unit_vector[2] * distance]

        return redeem_vector
    
    # 목표 지점에 도달할 때까지 거리 피드백 제공, 목표 지점에 가까워지면 이동을 중지
    def nav_distance_feedback(self, target_yaw):
        if self.target_type == "Main":
            sec = 0
        else:
            sec = 0.5
        i = 0
        send_data = Float32()
        while not self.nav.isTaskComplete():
            self.check_emergency_status()
            i = i + 1
            feedback = self.nav.getFeedback()
            self.current_path_publisher.publish(self.current_path_msg)
            if feedback and i % 30 == 0:
                self.get_logger().info("distance remaining: " + "{:.2f}".format(feedback.distance_remaining) + " meters.")
                send_data.data = feedback.distance_remaining
                
                if (feedback.distance_remaining <= 0.15 and feedback.distance_remaining != 0.0 and 0.1 > (self.my_yaw - target_yaw)) or Duration.from_msg(feedback.navigation_time) > Duration(seconds=40.0):
                    time.sleep(sec)
                    self.nav.cancelTask()
                    self.get_logger().info("cancel nav Task")

                    return
    
    def scan_callback(self, msg):
        min_distance = min(msg.ranges)
        if min_distance < 0.2:  # 장애물까지의 거리가 0.2m 이하일 경우
            self.get_logger().warn("Obstacle detected! Replanning path...")
            self.replan_path()

    def replan_path(self, scan_msg):
        self.nav.cancelTask()
        
        # 현재 위치와 목표 위치 가져오기
        current_pose = self.my_pose
        target_pose = self.POSE_DICT[self.current_task_location]
        
        # 장애물 감지
        min_distance = min(scan_msg.ranges)
        if min_distance < 0.15:  # 장애물까지의 거리가 0.2m 이하일 경우
            self.get_logger().warn("Obstacle detected! Replanning path using Bug2 algorithm...")

            # 장애물이 있는 방향을 찾음
            angle_increment = scan_msg.angle_increment
            angles = np.arange(scan_msg.angle_min, scan_msg.angle_max, angle_increment)
            min_index = np.argmin(scan_msg.ranges)
            obstacle_angle = angles[min_index]

            # 장애물 회피를 위해 현재 위치에서 회피 벡터 계산
            avoid_distance = 0.3  # 회피 거리
            avoid_pose = [
                current_pose[0] + avoid_distance * np.cos(obstacle_angle + np.pi / 2),
                current_pose[1] + avoid_distance * np.sin(obstacle_angle + np.pi / 2),
                current_pose[2]
            ]

            # 새로운 목표 위치로 설정
            self.move_pose(avoid_pose, self.my_yaw)
            
            # 장애물 회피 후 원래 목표 위치로 이동
            self.move_pose(target_pose, self.YAW_DICT[self.current_task_location])

def main(args=None): 
    rp.init(args=args)
    
    con = RobotControl()
    sub = My_Location(control=con)

    executor = MultiThreadedExecutor()
    executor.add_node(con)
    executor.add_node(sub)
    
    import sys
    if len(sys.argv) > 1:
        pose_name = sys.argv[1]
        if pose_name in con.POSE_DICT:
            con.follow_path(pose_name, LED=False)
        else:
            con.get_logger().error(f"Unknown pose name: {pose_name}")
    else:
        con.get_logger().info("No pose name provided.")
    
    try: 
        executor.spin()
    finally: 
        executor.shutdown()
        sub.destroy_node()
        con.destroy_node()
        rp.shutdown()

if __name__ == "__main__": 
    main()
