"""
robot_control.py
소품 픽앤플레이스 메인 제어 노드.

흐름:
  manager -> /pick_place_prop (커스텀 처리)
  -> object_detection /get_3d_position(SrvDepthPosition) 호출로 베이스 좌표 획득
  -> 픽앤플 모션 시퀀스(상공 접근 -> 하강 -> grip -> lift -> Place)

두산 코딩 규칙 준수:
  §1 헤더(DR_init id/model, 노드 후 __dsr__node 연결)
  §2 DSR_ROBOT2 import 는 노드 생성 이후 main 내부
  §4 비동기는 amovel + check_motion 루프 / 여기선 동기 movel 위주
  §5 좌표 .copy() 후 수정
  §10 JReady = posj([0, 0, 90, 0, 90, 0])
"""

import rclpy
import DR_init
from rclpy.node import Node

# ---- 두산 규칙 §1: 기본 설정 ----
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY = 60
ACC = 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

from doocut_interfaces.srv import SrvDepthPosition  # noqa: E402
from std_srvs.srv import Trigger                    # noqa: E402

NODE_NAME = "robot_control_node"

# 픽앤플 파라미터 (실로봇 측정 후 waypoints.yaml 로 이관 가능)
APPROACH_Z = 80.0       # 상공 접근 높이 (mm)
GRIP_DOWN_Z = 30.0      # 하강 깊이 (mm)
LIFT_Z = 80.0           # 집은 후 들어올림 (mm)
PLACE_POSX = [500.0, 200.0, 300.0, 0.0, 180.0, 0.0]   # 소품 전달 구역


class RobotControlNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME, namespace=ROBOT_ID)
        self.depth_cli = self.create_client(
            SrvDepthPosition, "/get_3d_position"
        )
        # manager 가 소품명을 넘겨 픽앤플 1회 수행시키는 트리거 서비스
        self.create_service(
            Trigger, "pick_place_ready", self._on_ready
        )
        self.get_logger().info("robot_control 노드 준비 완료")

    def _on_ready(self, request, response):
        response.success = True
        response.message = "robot_control ready"
        return response

    def request_position(self, target: str, timeout=5.0):
        """object_detection 에 소품 3D 좌표 요청."""
        if not self.depth_cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("/get_3d_position 서비스 없음")
            return None
        req = SrvDepthPosition.Request()
        req.target = target
        future = self.depth_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        res = future.result()
        if res is None or not res.success:
            self.get_logger().warn(f"'{target}' 좌표 획득 실패")
            return None
        return list(res.depth_position)


def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    DR_init.__dsr__node = node      # 두산 규칙 §1: 노드 연결

    # ---- 두산 규칙 §2: 노드 생성 이후 import ----
    try:
        from DSR_ROBOT2 import (
            movej, movel,
            wait,
            set_tool, set_tcp,
            get_current_posx,
            DR_BASE,
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        node.destroy_node()
        rclpy.shutdown()
        return

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    from robot_control.onrobot import RG2
    gripper = RG2()

    JReady = posj([0, 0, 90, 0, 90, 0])    # 두산 규칙 §10

    def go_home():
        movej(JReady, vel=VELOCITY, acc=ACC)

    def pick_and_place(target_xyz):
        """베이스 좌표계 소품 위치 -> 픽앤플 시퀀스."""
        # §5: 원본 보호 위해 copy 후 수정
        base = list(target_xyz)
        approach = posx([
            base[0], base[1], base[2] + APPROACH_Z,
            base[3], base[4], base[5],
        ])
        grip_pos = approach.copy()
        grip_pos[2] -= (APPROACH_Z + GRIP_DOWN_Z)

        gripper.open()
        movel(approach, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        movel(grip_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        gripper.close()
        wait(0.5)

        lift = grip_pos.copy()
        lift[2] += LIFT_Z
        movel(lift, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        go_home()

        place = posx(PLACE_POSX)
        movel(place, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        gripper.open()
        wait(0.5)
        go_home()

    # node.get_logger().info("초기 홈 위치 이동")
    go_home()

    # manager 가 별도 채널로 소품명을 넘기는 구조. 여기서는 노드를
    # 살려두고 spin (서비스 콜백/좌표요청 핸들링).
    node.pick_and_place = pick_and_place    # manager 연동 훅
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        gripper.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
