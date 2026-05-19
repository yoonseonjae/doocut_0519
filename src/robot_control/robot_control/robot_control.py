import os
import time
import sys
import yaml
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy
from rclpy.node import Node
import DR_init

from od_msg.srv import SrvDepthPosition
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
from robot_control.onrobot import RG

package_path = get_package_share_directory("robot_control")

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
JHOME_POS = [0, 0, 90, 0, 90, 0]
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
DEPTH_OFFSET = -5.0
MIN_DEPTH = 2.0

# ===== 웨이포인트 로드 (waypoints.yaml) =====
_wp_path = os.path.join(package_path, "resource", "waypoints.yaml")
with open(_wp_path, "r") as _f:
    _wp = yaml.safe_load(_f)
SCAN_POSITION  = _wp["scan_position"]["joint"]   # [j1..j6]
DROP_POSITION  = _wp["drop_position"]["joint"]    # [j1..j6] ← joint 방식으로 변경
HOME_POSITION  = _wp["home"]["joint"]             # [j1..j6]


DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import (
        movej, movel, get_current_posx, mwait, trans, check_motion
    )
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

########### Gripper Setup. Do not modify this area ############

gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)


########### Robot Controller ############


class RobotController(Node):
    def __init__(self):
        super().__init__("pick_and_place")
        self.init_robot()

        self.get_position_client = self.create_client(
            SrvDepthPosition, "/get_3d_position"
        )
        while not self.get_position_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_depth_position service...")
        self.get_position_request = SrvDepthPosition.Request()

        self.get_keyword_client = self.create_client(Trigger, "/get_keyword")
        while not self.get_keyword_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_keyword service...")
        self.get_keyword_request = Trigger.Request()

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
        """
        Converts 3D coordinates from the camera coordinate system
        to the robot's base coordinate system.
        """
        gripper2cam = np.load(gripper2cam_path)
        coord = np.append(np.array(camera_coords), 1)

        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)

        base2cam = base2gripper @ gripper2cam
        td_coord = np.dot(base2cam, coord)

        return td_coord[:3]

    # ============================================================
    # mwait() 대신 check_motion() 폴링 사용 (더 안전)
    # check_motion(): 0=Idle(완료), 1=Init, 2=Busy
    # ============================================================
    def wait_motion(self, timeout=30.0):
        time.sleep(0.2)
        start = time.time()
        while rclpy.ok():
            state = check_motion()
            if state == 0:
                self.get_logger().info("  -> motion done")
                return True
            if time.time() - start > timeout:
                self.get_logger().error(f"  -> motion TIMEOUT (state={state})")
                return False
            time.sleep(0.1)
        return False

    def wait_gripper(self, sec=2.0):
        """그리퍼 동작 완료 대기."""
        # 그리퍼 상태 폴링 + 안전 시간 확보
        start = time.time()
        while gripper.get_status()[0]:
            time.sleep(0.2)
            if time.time() - start > sec * 2:
                break
        time.sleep(sec)

    def robot_control(self):
        """
        메인 실행 루프.
        1. get_keyword 서비스 호출 → scene + props 파싱
        2. scan_position 이동 (카메라 스캔 자세)
        3. 소품별 반복: get_target_pos → pick_and_place_target → drop
        """
        self.get_logger().info("[robot_control] Calling get_keyword service...")
        self.get_logger().info("Say 'Hello Rokey' then describe the scene (예: 해변가)")

        get_keyword_future = self.get_keyword_client.call_async(self.get_keyword_request)
        rclpy.spin_until_future_complete(self, get_keyword_future)
        result = get_keyword_future.result()

        if not result or not result.success:
            self.get_logger().warn(f"get_keyword failed: {result.message if result else 'no response'}")
            return

        # 응답 형식: "beach umbrella bucket starfish"
        tokens = result.message.split()
        if len(tokens) < 2:
            self.get_logger().warn(f"No props in response: '{result.message}'")
            return

        scene = tokens[0]
        prop_list = tokens[1:]  # ["umbrella", "bucket", "starfish"]
        self.get_logger().info(f"Scene: '{scene}'  Props: {prop_list}")

        # ===== 스캔 위치로 이동 =====
        self.get_logger().info(f"[1/N] Moving to scan_position: {SCAN_POSITION}")
        movej(SCAN_POSITION, vel=VELOCITY, acc=ACC)
        self.wait_motion()

        # ===== 소품별 pick & deliver =====
        for idx, prop in enumerate(prop_list, start=1):
            self.get_logger().info(f"[prop {idx}/{len(prop_list)}] Target: '{prop}'")

            target_pos = self.get_target_pos(prop)  # 기존 메서드 재사용
            if target_pos is None:
                self.get_logger().warn(f"  → '{prop}' not detected, skipping.")
                continue

            self.get_logger().info(f"  → 3D pos: {target_pos}")
            self.pick_and_place_target(target_pos)  # 기존 메서드 재사용

            # ===== 사용자 전달 위치로 이동 → 드랍 =====
            self.get_logger().info(f"  → Moving to drop_position: {DROP_POSITION}")
            movej(DROP_POSITION, vel=VELOCITY, acc=ACC)
            self.wait_motion()

            self.get_logger().info("  → Drop: open gripper")
            gripper.open_gripper()
            self.wait_gripper(1.5)

            # 다음 소품을 위해 스캔 위치 복귀
            if idx < len(prop_list):
                self.get_logger().info("  → Return to scan_position for next prop")
                movej(SCAN_POSITION, vel=VELOCITY, acc=ACC)
                self.wait_motion()

        # ===== 완료: 홈으로 복귀 =====
        self.get_logger().info("[robot_control] All props delivered. Returning home.")
        self.init_robot()

    def get_target_pos(self, target):
        self.get_position_request.target = target
        self.get_logger().info("call depth position service with object_detection node")
        get_position_future = self.get_position_client.call_async(
            self.get_position_request
        )
        rclpy.spin_until_future_complete(self, get_position_future)

        if get_position_future.result():
            result = get_position_future.result().depth_position.tolist()
            self.get_logger().info(f"Received depth position: {result}")
            if sum(result) == 0:
                print("No target position")
                return None

            gripper2cam_path = os.path.join(
                package_path, "resource", "T_gripper2camera.npy"
            )
            robot_posx = get_current_posx()[0]
            td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)

            if td_coord[2] and sum(td_coord) != 0:
                td_coord[2] += DEPTH_OFFSET
                td_coord[2] = max(td_coord[2], MIN_DEPTH)

            target_pos = list(td_coord[:3]) + robot_posx[3:]
        return target_pos

    def init_robot(self):
        self.get_logger().info("[init_robot] move to JReady (home)")
        JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
        self.wait_motion()
        gripper.open_gripper()
        self.wait_gripper(1.5)

    def pick_and_place_target(self, target_pos):
        # ===== 1. 타겟 상공 80mm 안전 접근 =====
        approach_pos = target_pos.copy()
        approach_pos[2] += 80
        self.get_logger().info(f"[1/4] approach above target: {approach_pos}")
        movel(approach_pos, vel=VELOCITY, acc=ACC)
        if not self.wait_motion():
            self.get_logger().error("approach move failed")
            return

        # ===== 2. 타겟까지 하강 (10mm 더 깊게) =====
        target_pos[2] -= 30
        self.get_logger().info(f"[2/4] descend to target: {target_pos}")
        movel(target_pos, vel=VELOCITY, acc=ACC)
        if not self.wait_motion():
            self.get_logger().error("descend move failed")
            return
        time.sleep(0.3)

        # ===== 3. 그리퍼 닫기 (물건 잡기) =====
        self.get_logger().info("[3/4] close gripper")
        gripper.close_gripper()
        self.wait_gripper(2.5)

        # ===== 4. 80mm 들어올리기 =====
        target_pos_up = trans(target_pos, [0, 0, 80, 0, 0, 0]).tolist()
        self.get_logger().info(f"[4/4] lift up: {target_pos_up}")
        movel(target_pos_up, vel=VELOCITY, acc=ACC)
        if not self.wait_motion():
            self.get_logger().error("lift move failed")
            return

        # → 함수 종료 후 robot_control()에서 init_robot()을 호출 → 홈으로 복귀
        #   홈 복귀 시 gripper.open_gripper()로 물건이 떨어짐


def main(args=None):
    node = RobotController()
    while rclpy.ok():
        node.robot_control()
    rclpy.shutdown()
    node.destroy_node()


if __name__ == "__main__":
    main()