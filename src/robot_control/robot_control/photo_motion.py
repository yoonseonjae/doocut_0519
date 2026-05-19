import rclpy
import DR_init
import threading
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, GoalResponse
from doocut_interfaces.action import PhotoSession
from std_srvs.srv import Trigger

# 두산 로봇 필수 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# 속도 및 가속도를 낮춰 부드러운 움직임 확보
VELOCITY = 40 
ACC = 40

class PhotoMotionServer:
    def __init__(self, node):
        self.node = node
        self.trigger_event = threading.Event()
        self.task_running = False
        self.goal_handle = None
        self.result = None

        # 촬영 서비스 클라이언트 (manager_node의 캡처 서비스 호출용)
        self.cli_capture = self.node.create_client(Trigger, '/capture_service')

        # 홈 위치 및 정면 촬영 위치 (사용자 환경에 맞게 조정 필요)
        # 지침 10번: JReady = posj([0, 0, 90, 0, 90, 0])
        self.JReady = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        
        # 정면 촬영 위치 (기존 waypoints의 중심점 활용)
        self.photo_pose = [525.72, -113.62, 565.21, 65.53, -101.16, -87.38]

    def perform_task(self):
        """실제 로봇 동작 수행 스레드"""
        while rclpy.ok():
            if self.trigger_event.wait(timeout=1.0):
                if not self.task_running:
                    self.task_running = True
                    try:
                        from DSR_ROBOT2 import (movej, amovel, amovej, check_motion, posx, posj)
                        
                        def wait_for_motion():
                            time.sleep(0.2)
                            while check_motion() != 0:
                                time.sleep(0.1)
                                if not rclpy.ok(): return False
                            return True

                        self.node.get_logger().info("📷 촬영 시퀀스 시작: 정면 위치로 이동")
                        
                        # 1. 정면 촬영 포즈로 이동 (비동기)
                        amovel(self.photo_pose, vel=VELOCITY, acc=ACC)
                        wait_for_motion()

                        # 2. 캡처 서비스 호출
                        self.node.get_logger().info("📸 사진 캡처 중...")
                        if self.cli_capture.wait_for_service(timeout_sec=2.0):
                            req = Trigger.Request()
                            self.cli_capture.call_async(req)
                        time.sleep(1.0) # 캡처 대기

                        # 3. 홈 위치로 복귀 (비동기 movej)
                        self.node.get_logger().info("🏠 촬영 완료, 홈 위치로 복귀")
                        amovej(self.JReady, vel=VELOCITY, acc=ACC)
                        wait_for_motion()

                        # 결과 반환
                        res = PhotoSession.Result()
                        res.success = True
                        self.result = res
                        self.goal_handle.succeed()

                    except Exception as e:
                        self.node.get_logger().error(f"오류 발생: {e}")
                        self.goal_handle.abort()
                    finally:
                        self.task_running = False
                        self.trigger_event.clear()

def ros_spin_thread(node, server_obj):
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    def goal_callback(goal_request):
        return GoalResponse.ACCEPT

    def execute_callback(goal_handle):
        server_obj.goal_handle = goal_handle
        server_obj.trigger_event.set()
        while server_obj.trigger_event.is_set() and rclpy.ok():
            time.sleep(0.1)
        return server_obj.result

    action_server = ActionServer(
        node,
        PhotoSession,
        'photo_session', # 액션명 확인 필요
        execute_callback=execute_callback,
        goal_callback=goal_callback
    )
    executor.spin()

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("photo_motion_server", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    server_obj = PhotoMotionServer(node)
    
    # 작업 스레드와 ROS 스핀 스레드 분리 (지침 7번)
    t1 = threading.Thread(target=server_obj.perform_task)
    t2 = threading.Thread(target=ros_spin_thread, args=(node, server_obj))
    
    t1.start()
    t2.start()
    t1.join()
    t2.join()

if __name__ == "__main__":
    main()