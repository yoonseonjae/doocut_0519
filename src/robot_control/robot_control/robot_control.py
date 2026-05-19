#!/usr/bin/env python3
"""
robot_control.py
소품 픽앤플레이스 메인 제어 노드 (매니저 연동 완벽 수정본)
"""

import os
import time
import threading
import yaml
import rclpy
import DR_init
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# ---- 두산 규칙 §1: 기본 설정 ----
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

from doocut_interfaces.srv import SrvDepthPosition  # noqa: E402
from std_srvs.srv import Trigger                    # noqa: E402

NODE_NAME = "robot_control_node"

def _load_waypoints():
    """resource/waypoints.yaml 로드. share -> 소스트리 순으로 폴백."""
    candidates = []
    try:
        share = get_package_share_directory("robot_control")
        candidates.append(os.path.join(share, "resource", "waypoints.yaml"))
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    candidates.append(os.path.join(here, "..", "resource", "waypoints.yaml"))

    for path in candidates:
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}, path
    return {}, None


class RobotControlNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME, namespace=ROBOT_ID)
        self.depth_cli = self.create_client(
            SrvDepthPosition, "/get_3d_position"
        )
        
        # 🟢 [수정 핵심] 매니저 노드에서 찌를 수 있는 2개의 스위치(서비스) 개통
        self.create_service(Trigger, "pick_place_ready", self._on_ready)
        self.create_service(Trigger, "pick_place_service", self._on_pick)

        # 🟢 로봇 모션을 비동기로 처리하기 위한 이벤트 신호기
        self.ready_event = threading.Event()
        self.pick_event = threading.Event()

        # ---- waypoints.yaml 로드 ----
        cfg, path = _load_waypoints()
        if path:
            self.get_logger().info(f"waypoints.yaml 로드: {path}")
        else:
            self.get_logger().warn("waypoints.yaml 없음 - 코드 기본값 사용 (좌표 부정확 가능)")

        self.scan_posx = cfg.get("scan_posx", [-225.97, -21.78, 720.63, 1.47, -154.73, 85.71])
        self.place_posx = cfg.get("place_posx", [348.62, -183.20, 293.75, 89.83, -92.18, -90.08])
        self.home_joint = cfg.get("home_joint", [0, 0, 90, 0, 90, 0])

        pp = cfg.get("pick_params", {})
        self.approach_z = pp.get("approach_z", 80.0)
        self.grip_down_z = pp.get("grip_down_z", 30.0)
        self.lift_z = pp.get("lift_z", 80.0)

        motion = cfg.get("motion", {})
        self.velocity = motion.get("velocity", 60)
        self.acc = motion.get("acc", 60)

        self.get_logger().info(f"scan={self.scan_posx} place={self.place_posx} approach_z={self.approach_z}")
        self.get_logger().info("✅ robot_control 노드 통신 스위치 준비 완료")

    # 🟢 스위치 1번: Ready 호출 시 스레드 트리거
    def _on_ready(self, request, response):
        self.ready_event.set()
        response.success = True
        response.message = "스캔 위치(Ready) 이동 명령 수락"
        return response

    # 🟢 스위치 2번: Pick 호출 시 스레드 트리거
    def _on_pick(self, request, response):
        self.pick_event.set()
        response.success = True
        response.message = "픽업(Pick & Place) 시퀀스 명령 수락"
        return response

    # 🟢 [수정 핵심] ROS 충돌(Deadlock)을 방지하는 안전한 비전 좌표 획득 로직
    def request_position(self, target: str, timeout=5.0):
        if not self.depth_cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("/get_3d_position 비전 서비스 없음")
            return None
            
        req = SrvDepthPosition.Request()
        req.target = target
        future = self.depth_cli.call_async(req)
        
        # spin_until_future_complete 대신 스레드 안전하게 대기
        end_time = time.time() + timeout
        while not future.done() and time.time() < end_time and rclpy.ok():
            time.sleep(0.1)
            
        if future.done():
            res = future.result()
            if res and res.success:
                return list(res.depth_position)
                
        self.get_logger().warn(f"'{target}' 좌표 획득 실패 혹은 타임아웃")
        return None


def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    DR_init.__dsr__node = node      # 두산 규칙 §1: 노드 연결

    # ---- 두산 규칙 §2: 노드 생성 이후 import ----
    try:
        from DSR_ROBOT2 import (
            movej, movel, wait, set_tool, set_tcp, DR_BASE
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        node.destroy_node()
        rclpy.shutdown()
        return

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    try:
        from robot_control.onrobot import RG2
        gripper = RG2()
    except Exception as e:
        node.get_logger().error(f"그리퍼 초기화 실패: {e}")
        gripper = None

    JReady = posj(node.home_joint)    
    VELOCITY = node.velocity
    ACC = node.acc

    def go_home():
        movej(JReady, vel=VELOCITY, acc=ACC)

    def go_scan():
        movel(posx(node.scan_posx), vel=VELOCITY, acc=ACC, ref=DR_BASE)

    def pick_and_place(target_xyz):
        base = list(target_xyz)
        approach = posx([
            base[0], base[1], base[2] + node.approach_z,
            base[3], base[4], base[5],
        ])
        grip_pos = approach.copy()
        grip_pos[2] -= (node.approach_z + node.grip_down_z)

        if gripper: gripper.open()
        movel(approach, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        movel(grip_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        if gripper: gripper.close()
        wait(0.5)

        lift = grip_pos.copy()
        lift[2] += node.lift_z
        movel(lift, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        go_home()

        place = posx(node.place_posx)
        movel(place, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        if gripper: gripper.open()
        wait(0.5)
        go_home()

    # 🟢 [수정 핵심] 실제 로봇을 구동하는 독립 작업자(Worker) 스레드
    def robot_worker():
        node.get_logger().info("🤖 로봇 모션 제어 워커 스레드 대기 중...")
        go_home() # 시작 시 홈으로 초기화
        
        while rclpy.ok():
            # 1. 스캔 위치 이동 명령이 들어왔을 때
            if node.ready_event.is_set():
                node.ready_event.clear()
                node.get_logger().info("▶️ [명령 수신] 스캔(Ready) 위치로 이동합니다.")
                go_scan()
                
            # 2. 픽업 명령이 들어왔을 때
            if node.pick_event.is_set():
                node.pick_event.clear()
                node.get_logger().info("▶️ [명령 수신] 비전 좌표 요청 및 픽업 시작...")
                # 비전 인식 노드에 대상 좌표 요청
                target = node.request_position("all") 
                if target:
                    node.get_logger().info(f"✔️ 비전 좌표 수신 완료: {target}, 픽업 진행")
                    pick_and_place(target)
                else:
                    node.get_logger().error("❌ 비전 좌표 수신 실패로 픽업을 취소하고 홈으로 복귀합니다.")
                    go_home()
                    
            time.sleep(0.1)

    # 작업자 스레드 시작
    worker_thread = threading.Thread(target=robot_worker, daemon=True)
    worker_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if gripper: gripper.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()