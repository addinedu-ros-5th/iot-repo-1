import rclpy as rp
from rclpy.node import Node
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ros_package_msgs.srv import CommandString
from ros_package_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped
import threading
import time , asyncio
import ros_package.routes as routes

class RobotDriver(Node):
    def __init__(self):
        super().__init__('Robot_Driver')

        # 로봇 상태 나타내는 퍼블리셔 
        self.robot_state_pose_publisher = self.create_publisher(RobotState, '/robot_state', 10)
        
        # Robot_Driving/robot_command 서비스를 받아오는 서비스 서버 생성
        self.robot_command_server = self.create_service(CommandString, '/Robot_Driving/robot_command', self.robot_command_callback)
                
        self.patrol_command_server = self.create_service(CommandString, '/patrol_command', self.patrol_command_callback)

        # 기본적으로 navigator를 초기화
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # 순찰 상태 관리 Patrolling
        self.current_state = 'start'
        self.patrolling = True
        self.command_received = False
        self.current_segment_index = 0
        self.current_point = 0

        # 순찰 방향을 위한 플래그
        self.forward_patrol = True

        # 순찰 경로 정의
              
        self.route_forward_segments = routes.route_forward_segments
        
        self.route_reverse_segments = routes.route_reverse_segments
        
        self.route_description = routes.route_description
        
        self.route_forward_guide_segments = routes.route_forward_guide_segments
        
        self.route_reverse_guide_segments = routes.route_reverse_guide_segments
        
        self.route_return_forward = routes.route_return_forward
        self.route_return_reverse = routes.route_return_reverse
        self.route_forward_return_segments = routes.route_forward_return_segments
        self.route_reverse_return_segments = routes.route_reverse_return_segments
        self.route_forward_guide_last = routes.route_forward_guide_last
        self.route_reverse_guide_last = routes.route_reverse_guide_last
                
        self.set_initial_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        
        self.goal_poses_art = ['아메리칸 슈렉','2','자가격리 중인 예수','바트심슨의 절규','5','스쿠터 탄 나폴레옹'] # 작품위치에 따른 작품명
        self.current_art = None
        # 스타트 신호
        self.publish_robot_state()

        # 1번 웨이포인트로 이동
        self.patrol_work()

        # 멈춤상태 신호
        self.publish_robot_state()
        
    def publish_robot_state(self):
        msg = RobotState()
        msg.command = self.current_state

        self.robot_state_pose_publisher.publish(msg)
        
    def set_goal_pose(self, x, y, z, qx, qy, qz, qw):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z
        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw
        return goal_pose
    
    def set_initial_pose(self, x, y, z, qx, qy, qz, qw):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = x
        initial_pose.pose.position.y = y
        initial_pose.pose.position.z = z
        initial_pose.pose.orientation.x = qx
        initial_pose.pose.orientation.y = qy
        initial_pose.pose.orientation.z = qz
        initial_pose.pose.orientation.w = qw
        self.navigator.setInitialPose(initial_pose)

    def follow_route_segment(self, current_route_segments, segment_index, max_retries=3):
        current_route = current_route_segments[segment_index]
        self.get_logger().info('segment_index : {}'.format(segment_index))

        for pt in current_route:
            if isinstance(pt, (list, tuple)) and len(pt) == 7:                
                goal_pose = self.set_goal_pose(*pt)
                self.navigator.goToPose(goal_pose)
                retries = 0
                
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()


            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                if current_route_segments == self.route_forward_segments:
                    self.current_point = segment_index
                    self.get_logger().info('Reached goal: ({}, {}), current_point: {}'.format(pt[0], pt[1], (self.current_point + 1)))
                elif current_route_segments == self.route_reverse_segments:
                    self.current_point = 4 - segment_index
                    self.get_logger().info('Reached goal: ({}, {}), current_point: {}'.format(pt[0], pt[1], (self.current_point + 1)))
                    
                if self.current_point == 5 :
                    self.forward_patrol = False
                elif self.current_point == 0:
                    self.forward_patrol = True
                
            elif result == TaskResult.CANCELED:
                self.get_logger().info('Goal was canceled, exiting.')
                return
            elif result == TaskResult.FAILED:
                retries += 1
                if retries >= max_retries:
                    self.get_logger().info('Failed to reach goal: ({}, {}), max retries reached, exiting.'.format(pt[0], pt[1]))
                    break
                self.get_logger().info('Failed to reach goal: ({}, {}), retrying...'.format(pt[0], pt[1]))


    def patrol_work(self):
        self.current_state = 'Patrolling'
        self.publish_robot_state()

        # 현재 경로와 방향 선택
        current_route_segments = self.route_forward_segments if self.forward_patrol else self.route_reverse_segments
        
        # 경로의 현재 세그먼트 따라가기
        self.follow_route_segment(current_route_segments, self.current_segment_index)
        # 세그먼트 인덱스 업데이트
        self.current_segment_index += 1
        self.get_logger().info('current_segment_index : {}'.format(self.current_segment_index))

        # 경로의 끝에 도달하면 순찰 방향을 반대로 전환하고 인덱스 초기화
        if self.current_segment_index >= len(current_route_segments):
            self.current_segment_index = 0

        art_point = self.current_point
        # 상태를 업데이트
        self.art = self.goal_poses_art[art_point]  # 수정된 부분
        self.current_state = 'arrive at {}'.format(self.art)  # 작품명에 도착
        self.get_logger().info('current_state: {}'.format(self.current_state))
        self.get_logger().info('forward_patrol: {}'.format(self.forward_patrol))


    def patrol_command_callback(self, request, response):

        self.get_logger().info('Patrol Command Server started')

        if request.command == "patrol":
            self.get_logger().info('Received patrol')
            response.success = True
            response.message = 'patrol'   

            self.current_state = 'Patrolling'

            self.publish_robot_state()
        
            self.patrol_work()

            self.publish_robot_state()

        return response


    def robot_command_callback(self, request, response):
        
        self.get_logger().info('Robot Command Server started')

        if request.command == "human_detect":
            self.get_logger().info('Received human_detect')
            response.success = True
            response.message = 'human_detect'   

            self.current_state = 'Human Detection'

            self.publish_robot_state()

            self.handle_human_detect()


        elif request.command == "description":
            self.get_logger().info('Received description')
            response.success = True
            response.message = 'description'

            self.current_state = 'Description'

            self.publish_robot_state()


        elif request.command == "guide":
            self.get_logger().info('Received guide')
            response.success = True
            response.message = 'guide'

            self.current_state = 'Guiding'

            self.publish_robot_state()

            self.handle_guide(request.description)

        elif request.command == "comeback":
            self.get_logger().info('Received comeback')
            response.success = True
            response.message = 'comeback'

            self.current_state = 'comeback_to_patrol'

            self.publish_robot_state()

            self.comeback_to_patrol()
            
            self.patrol_work()

            self.publish_robot_state()

        else:
            self.get_logger().error('Received Unknown')
            response.success = False
            response.message = 'Unknown'


        return response
    
    def get_description_index(self, current_point):
        if current_point == 0:
            return 0
        elif current_point == 2:
            return 1
        elif current_point == 3:
            return 2
        elif current_point == 5:
            return 3
           
    def get_current_point(self, description_index):
        if description_index == 0:
            return 0
        elif description_index == 1:
            return 2
        elif description_index == 2:
            return 3
        elif description_index == 3:
            return 5
        else:
            raise ValueError("Invalid description index")
    
    def handle_human_detect(self):
        self.get_logger().info('human_detect.')
        self.description_index = self.get_description_index(self.current_point)      
        self.follow_route_segment(self.route_description, self.description_index)


    def handle_guide(self, description):
        self.get_logger().info('Robot guiding to description location.')

        # 현재 위치 인덱스와 목표 작품 인덱스 확인
        self.art_index = self.goal_poses_art.index(description)
        self.get_logger().info('art_index: {}'.format(self.art_index))

        # 경로의 방향 결정 (정방향 또는 역방향)
        if self.art_index > self.current_point:
            current_route_segments = self.route_forward_guide_segments
            current_return_segments = self.route_return_forward
            current_guide_last = self.route_forward_guide_last
            start_index = self.current_point + 1
            end_index = self.art_index
        else:
            current_route_segments = self.route_reverse_guide_segments
            current_return_segments = self.route_return_reverse
            current_guide_last = self.route_reverse_guide_last

            start_index = 5 - self.current_point
            end_index = 4 - self.art_index
        self.get_logger().info('start_index: {}'.format(start_index))
        self.get_logger().info('end_index: {}'.format(end_index))
        # 주행 경로를 따라 순차적으로 주행
        
        self.follow_route_segment(current_return_segments, self.description_index)
        
        for index in range(start_index, end_index):        
            self.get_logger().info('index: {}'.format(index))
            self.follow_route_segment(current_route_segments, index)

        self.follow_route_segment(current_guide_last, end_index)

        self.current_point = self.art_index
        self.get_logger().info('self.current_point: {}'.format(self.current_point))
        self.description_index = self.get_description_index(self.current_point)      

        # 도착 상태로 업데이트
        self.current_state = 'Arrived at {}'.format(description)
        self.publish_robot_state()


    def comeback_to_patrol(self):
        self.get_logger().info('Returning to patrol route.')
        
        # 주행 경로를 따라 주행
        if self.forward_patrol == True :
            current_route_segments = self.route_return_forward
        else :
            current_route_segments = self.route_return_reverse
            
        self.follow_route_segment(current_route_segments, self.description_index)
            
        self.get_logger().info('Returned to patrol route at segment index: {}'.format(self.description_index))
        self.current_point = self.get_current_point(self.description_index)

        if self.current_point == 5: 
            self.current_segment_index = 0
            self.follow_route_segment(self.route_reverse_return_segments, self.current_segment_index)
            self.current_point = 4

            self.current_segment_index += 1

        elif self.current_point == 0:
            self.current_segment_index = 1
            self.follow_route_segment(self.route_forward_return_segments, self.current_segment_index)
            self.current_point = 1
            self.current_segment_index += 1

        elif self.current_point == 2:
            if self.forward_patrol == True :
                self.current_segment_index = 3
                self.follow_route_segment(self.route_forward_return_segments, self.current_segment_index)
                self.current_point = 3
                self.current_segment_index += 1

            else :
                self.current_segment_index = 2
                self.follow_route_segment(self.route_reverse_return_segments, self.current_segment_index)
                self.current_segment_index += 1
                self.follow_route_segment(self.route_reverse_return_segments, self.current_segment_index)
                self.current_segment_index += 1
                self.current_point = 1



        elif self.current_point == 3:
            if self.forward_patrol == True :
                self.current_segment_index = 3
                self.follow_route_segment(self.route_forward_return_segments, self.current_segment_index)
                self.current_segment_index += 1
                self.follow_route_segment(self.route_forward_return_segments, self.current_segment_index)
                self.current_segment_index += 1
                self.current_point = 4
            else :
                self.current_segment_index = 2
                self.follow_route_segment(self.route_reverse_return_segments, self.current_segment_index)
                self.current_segment_index += 1
                self.current_point = 1

                
        art_point = self.current_point
        self.get_logger().info('art_point: {}'.format(art_point))   
        # 상태를 업데이트
        self.art = self.goal_poses_art[art_point]  # 수정된 부분
        self.current_state = 'arrive at {}'.format(self.art)  # 작품명에 도착
        self.get_logger().info('current_state: {}'.format(self.current_state))   

        self.get_logger().info('current_segment_index : {}'.format(self.current_segment_index))
        self.get_logger().info('forward_patrol : {}'.format(self.forward_patrol))

            

def main(args=None):
    rp.init(args=args)

    robot_driving = RobotDriver()

    rp.spin(robot_driving)

    robot_driving.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()