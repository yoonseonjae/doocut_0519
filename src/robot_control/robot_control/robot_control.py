"""
robot_control.py
소품 픽앤플레이스 메인 제어 노드.

흐름:
  manager -> /pick_place_prop (커스텀 처리)
  -> object_detection /get_3d_position(SrvDepthPosition) 호출로 베이스 좌표 획득
  -> 픽앤플 모션 시퀀스(스캔 -> 상공 접근 -> 하강 -> grip -> lift -> Place)

좌표는 resource/waypoints.yaml 에서 로드한다 (하드코딩 제거).

두산 코딩 규칙 준수:
  §1 헤더(DR_init id/model, 노드 후 __dsr__node 연결)
  §2 DSR_ROBOT2 import 는 노드 생성 이후 main 내부
  §5 좌표 .copy() 후 수정
  §10 JReady = posj([0, 0, 90, 0, 90, 0])
"""

import os

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
        self.create_service(
            Trigger, "pick_place_ready", self._on_ready
        )

        # ---- waypoints.yaml 로드 ----
        cfg, path = _load_waypoints()
        if path:
            self.get_logger().info(f"waypoints.yaml 로드: {path}")
        else:
            self.get_logger().warn(
                "waypoints.yaml 없음 - 코드 기본값 사용 (좌표 부정확 가능)")

        self.scan_posx = cfg.get(
            "scan_posx", [-225.97, -21.78, 720.63, 1.47, -154.73, 85.71])
        self.place_posx = cfg.get(
            "place_posx", [348.62, -183.20, 293.75, 89.83, -92.18, -90.08])
        self.home_joint = cfg.get("home_joint", [0, 0, 90, 0, 90, 0])

        pp = cfg.get("pick_params", {})
        self.approach_z = pp.get("approach_z", 80.0)
        self.grip_down_z = pp.get("grip_down_z", 30.0)
        self.lift_z = pp.get("lift_z", 80.0)

        motion = cfg.get("motion", {})
        self.velocity = motion.get("velocity", 60)
        self.acc = motion.get("acc", 60)

        self.get_logger().info(
            f"scan={self.scan_posx} place={self.place_posx} "
            f"approach_z={self.approach_z}")
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

    JReady = posj(node.home_joint)    # 두산 규칙 §10 (yaml 로드)
    VELOCITY = node.velocity
    ACC = node.acc

    def go_home():
        movej(JReady, vel=VELOCITY, acc=ACC)

    def go_scan():
        """소품 스캔 자세로 이동 (카메라가 선반을 내려다봄)."""
        movel(posx(node.scan_posx), vel=VELOCITY, acc=ACC, ref=DR_BASE)

    def pick_and_place(target_xyz):
        """베이스 좌표계 소품 위치 -> 픽앤플 시퀀스."""
        # §5: 원본 보호 위해 copy 후 수정
        base = list(target_xyz)
        approach = posx([
            base[0], base[1], base[2] + node.approach_z,
            base[3], base[4], base[5],
        ])
        grip_pos = approach.copy()
        grip_pos[2] -= (node.approach_z + node.grip_down_z)

        gripper.open()
        movel(approach, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        movel(grip_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        gripper.close()
        wait(0.5)

        lift = grip_pos.copy()
        lift[2] += node.lift_z
        movel(lift, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        go_home()

        place = posx(node.place_posx)
        movel(place, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        gripper.open()
        wait(0.5)
        go_home()

    go_home()

    # manager 연동 훅
    node.go_scan = go_scan
    node.pick_and_place = pick_and_place
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