import rclpy as rp
import os
import time
import math
import numpy as np
from rclpy.node import Node
from copy import deepcopy
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from rclpy.executors import MultiThreadedExecutor

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

from task_msgs.srv import AllocateTask
from task_msgs.srv import ArucoCommandResponse
from task_msgs.srv import ArucoCommand

from task_msgs.msg import TaskCompletion
from task_msgs.msg import CurrentPath
from task_msgs.msg import RobotStatus
from task_msgs.msg import OutTask

ID = os.getenv('ROS_DOMAIN_ID', 'Not set')

class My_Location(Node) : 
    def __init__(self, control):
        super().__init__("robot_loc")
        self.control = control
        
        self.obstacle_1 = [[1, 1],[1, 1]]
        self.obstacle_2 = [[1, 1],[1, 1]]
        
        # publisher
        self.publisher_out_task = self.create_publisher(
            OutTask,
            "/out_task",
            10
        )

# -----------------------------------------------------------------------------

        # self.publisher_current_path = self.create_publisher(
        #     CurrentPath,
        #     "/obstacle",
        #     10
        # )
        
# ----------------------------------------------------------------------------
        
        # subscriber
        self.subcription_amclpose = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.current_pose,
            10
        )

        self.subscription_emergency_stop = self.create_subscription(
            Empty,
            "/emergency_stop",
            self.emergency_button_is_clicked,
            10
        )
# ------------------------------------------------------------------------------
        # self.subscription_current_path = self.create_subscription(
        #     CurrentPath,
        #     "/obstacle_1",
        #     self.renew_map_status_1,
        #     10
        # )

        # self.subscription_current_path = self.create_subscription(
        #     CurrentPath,
        #     "/obstacle_2",
        #     self.renew_map_status_2,
        #     10
        # )
        
        self.timer = self.create_timer(5.0, self.renew_out_task_data)
        self.path_timer = self.create_timer(0.1, self.send_path_status)
# -------------------------------------------------------------------------------        
    # 장애물 1의 위치 데이터를 갱신, send_map_status를 호출해서 장애물 상태 퍼블리시
    # def renew_map_status_1(self, data):
        
    #     self.obstacle_1[0][0] = data.start_x
    #     self.obstacle_1[0][1] = data.start_y
    #     self.obstacle_1[1][0] = data.end_x
    #     self.obstacle_1[1][1] = data.end_y

    #     self.send_map_status()

    # # 장애물 2의 위치 데이터를 갱신, send_map_status를 호출해서 장애물 상태 퍼블리시
    # def renew_map_status_2(self, data):
    #     self.obstacle_2[0][0] = data.start_x
    #     self.obstacle_2[0][1] = data.start_y
    #     self.obstacle_2[1][0] = data.end_x
    #     self.obstacle_2[1][1] = data.end_y

    #     self.send_map_status()

    # map_data를 갱신하여 장애물 정보를 반영
    # def send_map_status(self):
    #     map_data = deepcopy(self.control.is_passable_list)
    #     if self.obstacle_1[0][0] == -1:
    #         map_data[self.obstacle_1[0][0]][self.obstacle_1[0][1]] = True
    #         map_data[self.obstacle_1[1][0]][self.obstacle_1[1][1]] = True
    #     else:
    #         map_data[self.obstacle_1[0][0]][self.obstacle_1[0][1]] = False
    #         map_data[self.obstacle_1[1][0]][self.obstacle_1[1][1]] = False

    #     if self.obstacle_2[0][0] == -1:
    #         map_data[self.obstacle_2[0][0]][self.obstacle_2[0][1]] = True
    #         map_data[self.obstacle_2[1][0]][self.obstacle_2[1][1]] = True
    #     else:
    #         map_data[self.obstacle_2[0][0]][self.obstacle_2[0][1]] = False
    #         map_data[self.obstacle_2[1][0]][self.obstacle_2[1][1]] = False
        
    #     self.control.current_is_passable_list = deepcopy(map_data)

# -------------------------------------------------------------------------------

    # 경로 상태 및 작업 할당 데이타 퍼블리시
    # def send_path_status(self):
    #     self.publisher_current_path.publish(self.control.current_path_msg)
        
    # def renew_out_task_data(self):
    #     msg = OutTask()
    #     msg.location = self.control.current_task_location
    #     msg.product = self.control.item
    #     msg.count = self.control.quantity
        
    #     self.publisher_out_task.publish(msg)

    # 현재 위치와 방향 데이터 갱신
    def current_pose(self, data):
        self.control.my_pose[0] = data.pose.pose.position.x
        self.control.my_pose[1] = data.pose.pose.position.y

        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w
        
        roll, pitch, yaw = self.euler_from_quaternion(x, y, z, w)
        self.control.my_yaw  = yaw

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
    
class RobotControl(Node) : 
    def __init__(self) : 
        super().__init__("robot_control")
        self.get_logger().info("start robot_control")
        self.nav = BasicNavigator()
        
        self.current_task_location  = ""
        self.tasking                = False
        self.next_out               = False
        self.marker_service_done    = False
        
        self.target_type    = ""
        self.task_status    = ""
        self.task_id        = ""
        self.item           = ""
        self.quantity       = 0
        self.my_pose        = [0.0, 0.0, 0.0]
        self.my_yaw         = 0
        
        self.current_path_msg = CurrentPath()
        
        self.server = self.create_service(
            AllocateTask, 
            f"/allocate_task_{ID}", 
            self.task_callback
        )

        
        self.task_completion_publisher = self.create_publisher(
            TaskCompletion,
            "/task_completion",
            10
        )

        self.robot_status_publisher = self.create_publisher(
            RobotStatus,
            "/robot_status",
            10
        )

        self.current_path_publisher = self.create_publisher(
            CurrentPath,
            "/current_path",
            10
        )
        
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.arucomarker_client = self.create_client(
            ArucoCommand, "/aruco_control"
        )
        
    # 맵의 좌표를 미리 지정하는 단계
    # pose x, y, z
        # 추후 json or yaml로 변경 필요
        self.POSE_DICT = {
            "I1" : [0.4, -1.425, 0.0],  "I2" : [0.4, -0.775, 0.0],  "I3" : [0.4, -0.625, 0.0],
            "O1" : [1.45, 1.06, 0.0],   "O2" : [1.45, 0.66, 0.0],   "O3" : [1.45, 0.46, 0.0],
            "P1" : [0.4, 1.16, 0.0],    "P2" : [0.4, 0.58, 0.0],    "P3" : [0.4, 0.0, 0.0],
            "R1" : [1.45, -1.2, 0.0],   "R2" : [1.45, -0.9, 0.0],
            "A1" : [0.8, 0.3, 0.0],    "A1_2" : [0.8, 0.3, 0.0], 
            "A2" : [1.02, 0.3, 0.0],    "A2_2" : [1.02, 0.3, 0.0],
            "B1" : [0.8, -0.5, 0.0],   "B1_2" : [0.8, -0.5, 0.0], 
            "B2" : [1.02, -0.5, 0.0],   "B2_2" : [1.02, -0.5, 0.0],
            "C1" : [0.8, -1.2, 0.0],   "C1_2" : [0.8, -1.2, 0.0], 
            "C2" : [1.02, -1.2, 0.0],   "C2_2" : [1.02, -1.2, 0.0]
        }
    # 그 위치에서의 자세 및 방향
        self.YAW_DICT = {
            "I1" : 3.14,    "I2" : 3.14,    "I3" : 3.14,
            "O1" : 0.0,     "O2" : 0.0,     "O3" : 0.0,
            "P1" : 3.14,    "P2" : 3.14,    "P3" : 3.14,
            "A1" : 1.57,    "A1_2" : 1.57, 
            "A2" : 1.57,    "A2_2" : 1.57,
            "B1" : 1.57,    "B1_2" : 1.57, 
            "B2" : 1.57,    "B2_2" : 1.57, 
            "C1" : 1.57,    "C1_2" : 1.57, 
            "C2" : 1.57,    "C2_2" : 1.57, 
            "R1" : 0.0,     "R2" : 0.0
        }

        self.declare_lists()
        
        
        self.send_robot_status_topics()
        self.get_logger().info("control is ready")


    def declare_lists(self):
        self.PATH_LIST = [
            [[0.3, -1.1, 0.0],  [0.8, -1.1, 0.0],  [1.2, -1.1, 0.0],  [1.5, -1.1, 0.0]],
            [[0.3, -0.75, 0.0], [0.8, -0.75, 0.0], [1.2, -0.75, 0.0], [1.5, -0.75, 0.0]],
            [[0.3, -0.4, 0.0],  [0.8, -0.4, 0.0],  [1.2, -0.4, 0.0],  [1.5, -0.4, 0.0]],
            [[0.3, 0.0, 0.0],   [0.8, 0.0, 0.0],   [1.2, 0.0, 0.0],   [1.5, 0.0, 0.0]],
            [[0.3, 0.4, 0.0],   [0.8, 0.4, 0.0],   [1.2, 0.4, 0.0],   [1.5, 0.4, 0.0]],
            [[0.3, 0.75, 0.0],  [0.8, 0.75, 0.0],  [1.2, 0.75, 0.0],  [1.5, 0.75, 0.0]],
            [[0.3, 1.0, 0.0],   [0.8, 1.0, 0.0],   [1.2, 1.0, 0.0],   [1.5, 1.0, 0.0]]
        ]
        
        self.X_LIST = []
        self.Y_LIST = []
        for i in self.PATH_LIST[0]:
            self.X_LIST.append(i[0])
        for i in self.PATH_LIST:
            self.Y_LIST.append(i[0][1])

        # 이동 가능 지점, 불가능 지점을 지정
        self.is_passable_list   = [[True for _ in range(len(self.PATH_LIST[0]))] for _ in range(len(self.PATH_LIST))]
        not_passable_index_list = [[1, 1], [1, 2], [3, 1], [3, 2], [5, 1], [5, 2]]
        
        for i in not_passable_index_list:
            self.is_passable_list[i[0]][i[1]] = False

        # 현재 이동 가능 지점에 복사
        self.current_is_passable_list = deepcopy(self.is_passable_list)
        
    def check_emergency_status(self):
        while self.task_status == "emergency":
            try:
                self.nav.cancelTask()
            except:
                pass
            self.get_logger().info("emergency!!!!!!!!!!")
            time.sleep(1)

    def send_robot_status_topics(self):
        msg = RobotStatus()

        msg.robot_id = ID
        msg.robot_status = "available"

        for i in range(20):
            self.robot_status_publisher.publish(msg)
            time.sleep(0.1)
            
    def task_callback(self, req, res) :
        self.get_logger().info(f"task_list: {req.location}, {req.task_type}, {req.lift}")
        if not self.tasking:
            try:
                self.item                   = req.item
                self.tasking                = True
                self.task_id                = req.task_id
                self.quantity               = req.quantity
                self.task_status            = req.task_type
                self.current_task_location  = req.location
                
                res.success     = self.follow_path(self.current_task_location, req.lift)
                self.tasking    = False

                self.send_complete_task_topics()

            except Exception as e:
                self.tasking = False
                self.get_logger().error(f"{e}")
                
        else:
            res.success = False

        return res
    
    # 경로 계획을 시작함
    def follow_path(self, pose_name, lift):
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

    def move_pose(self, target_pose, yaw) :
        q = self.euler_to_quaternion(yaw=yaw)

        self.get_logger().info(f"pose : {target_pose} yaw : {yaw}")
        goal_pose       = PoseStamped()
        redeem_vector   = self.redeem_pose(target_pose, 0.25)
        
        goal_pose.header.frame_id   = 'map'
        goal_pose.header.stamp      = self.nav.get_clock().now().to_msg()

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
    
    # 목표 지점에 도달할 때까지 거리 피드벡 제공, 목표 지점에 가까워지면 이동을 중지
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
        current_yaw = self.my_yaw
        angle_increment = scan_msg.angle_increment
        angles = np.arange(scan_msg.angle_min, scan_msg.angle_max, angle_increment)
        
        # 장애물이 있는 방향을 찾음
        min_index = np.argmin(scan_msg.ranges)
        obstacle_angle = angles[min_index]
        
        # 장애물을 회피하기 위해 yaw 방향을 변경
        avoid_angle = obstacle_angle + np.pi / 4  # 45도 회전 (조정 가능)
        
        # 새로운 목표 yaw 설정
        new_yaw = current_yaw + avoid_angle
        new_yaw = np.arctan2(np.sin(new_yaw), np.cos(new_yaw))  # [-pi, pi] 범위로 변환

        # 새로운 목표 위치 설정 (현재 위치에서 약간 이동)
        new_target_pose = [self.my_pose[0] + 0.5 * np.cos(new_yaw), self.my_pose[1] + 0.5 * np.sin(new_yaw), 0.0]
        
        self.move_pose(new_target_pose, new_yaw)

def main(args = None) : 
    rp.init(args=args)
    
    con = RobotControl()
    sub = My_Location(control = con)

    excutor = MultiThreadedExecutor()
    excutor.add_node(con)
    excutor.add_node(sub)
    
    try : 
        excutor.spin()
    finally : 
        excutor.shutdown()
        sub.destroy_node()
        con.destroy_node()
        rp.shutdown()

    


if __name__ == "__main__" : 
    main()
