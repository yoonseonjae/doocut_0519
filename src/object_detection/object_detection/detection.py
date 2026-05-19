"""
detection.py
SrvDepthPosition 서비스 서버 (/get_3d_position).

레퍼지토리 공통 패턴 (PART1 핵심 5패턴 중 (1),(2)) 완전 구현:
  (2) 픽셀->카메라 3D 복원 (핀홀):
        cam_x=(px-ppx)*z/fx ; cam_y=(py-ppy)*z/fy ; cam_z=z
  (1) 카메라->베이스 변환 (transform_to_base):
        R = Rotation.from_euler("ZYZ",[rx,ry,rz],deg).as_matrix()
        base2gripper = pose_matrix(현재 posx)
        base2cam = base2gripper @ T_gripper2camera   # npy 로드
        base_xyz = base2cam @ [cam_x,cam_y,cam_z,1]

두산 코딩 규칙 §1/§2 준수. DSR API 는 get_current_posx 만 사용.
"""

import os

import numpy as np
import rclpy
import DR_init
from rclpy.node import Node

# ---- 두산 규칙 §1 ----
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

from doocut_interfaces.srv import SrvDepthPosition          # noqa: E402
from object_detection.yolo import YoloDetector, resolve_model_path  # noqa

NODE_NAME = "detection_node"

# 레퍼지토리 공통 보정 상수
DEPTH_OFFSET = -5.0      # mm
MIN_DEPTH = 2.0          # mm


def pose_to_matrix(posx_vals):
    """posx [x,y,z,rx,ry,rz](mm/deg) -> 4x4 동차행렬 (ZYZ 오일러)."""
    from scipy.spatial.transform import Rotation
    x, y, z, rx, ry, rz = posx_vals
    R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


class DetectionNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME, namespace=ROBOT_ID)

        # ---- YOLO 모델 ----
        self.detector = YoloDetector(conf=0.5)

        # ---- 핸드아이 캘리브레이션 행렬 로드 ----
        self.T_g2c = self._load_handeye()

        # ---- RealSense 파이프라인 ----
        self.pipeline = None
        self.intrin = None
        self._init_realsense()

        self.srv = self.create_service(
            SrvDepthPosition, "/get_3d_position", self.handle_request
        )
        self.get_logger().info("SrvDepthPosition 서버 준비 (/get_3d_position)")

    def _load_handeye(self):
        path = resolve_model_path(None, "object_detection",
                                  "T_gripper2camera.npy")
        if os.path.exists(path):
            try:
                T = np.load(path)
                self.get_logger().info(f"핸드아이 행렬 로드: {path}")
                return T
            except Exception as e:
                self.get_logger().error(f"핸드아이 로드 실패: {e}")
        self.get_logger().warn(
            "T_gripper2camera.npy 없음 - 캘리브레이션 수행 후 resource/ 배치 필요. "
            "임시로 단위행렬 사용 (좌표 부정확)."
        )
        return np.eye(4)

    def _init_realsense(self):
        try:
            import pyrealsense2 as rs
            self.pipeline = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            profile = self.pipeline.start(cfg)
            self.align = rs.align(rs.stream.color)
            color_stream = profile.get_stream(rs.stream.color)
            self.intrin = color_stream.as_video_stream_profile().get_intrinsics()
            self.get_logger().info("RealSense 시작")
        except Exception as e:
            self.get_logger().warn(f"RealSense 비활성 (모의 모드): {e}")
            self.pipeline = None

    def _grab_frames(self):
        """(color_image, depth_frame, rs) 반환. 실패 시 (None,None,None)."""
        if self.pipeline is None:
            return None, None, None
        import pyrealsense2 as rs
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)
        depth = aligned.get_depth_frame()
        color = aligned.get_color_frame()
        if not depth or not color:
            return None, None, None
        return np.asanyarray(color.get_data()), depth, rs

    def _pixel_to_cam(self, px, py, depth_frame, rs):
        """핀홀 역투영 -> 카메라 좌표 (mm)."""
        # 중앙 3x3 중앙값으로 z 안정화
        zs = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                d = depth_frame.get_distance(
                    int(px) + dx, int(py) + dy)
                if d > 0:
                    zs.append(d)
        if not zs:
            return None
        z_m = float(np.median(zs))
        z = z_m * 1000.0 + DEPTH_OFFSET     # m -> mm + 보정
        if z < MIN_DEPTH:
            return None
        fx, fy = self.intrin.fx, self.intrin.fy
        ppx, ppy = self.intrin.ppx, self.intrin.ppy
        cam_x = (px - ppx) * z / fx
        cam_y = (py - ppy) * z / fy
        cam_z = z
        return np.array([cam_x, cam_y, cam_z, 1.0])

    def _cam_to_base(self, cam_xyz1):
        """transform_to_base: base2gripper @ T_g2c @ cam_xyz."""
        try:
            from DSR_ROBOT2 import get_current_posx, DR_BASE
            cur, _ = get_current_posx(ref=DR_BASE)
        except Exception as e:
            self.get_logger().error(f"get_current_posx 실패: {e}")
            return None
        base2gripper = pose_to_matrix(cur)
        base2cam = base2gripper @ self.T_g2c
        base_xyz = base2cam @ cam_xyz1
        return base_xyz[:3], cur

    def handle_request(self, request, response):
        target = request.target
        color, depth, rs = self._grab_frames()
        if color is None:
            response.success = False
            response.message = "no camera frame"
            response.depth_position = []
            return response

        dets = self.detector.detect(color)
        cand = [d for d in dets if d["name"] == target]
        if not cand:
            response.success = False
            response.message = f"'{target}' not detected"
            response.depth_position = []
            return response

        best = max(cand, key=lambda d: d["conf"])
        cam = self._pixel_to_cam(best["cx"], best["cy"], depth, rs)
        if cam is None:
            response.success = False
            response.message = "invalid depth"
            response.depth_position = []
            return response

        result = self._cam_to_base(cam)
        if result is None:
            response.success = False
            response.message = "base transform failed"
            response.depth_position = []
            return response

        base_xyz, cur_pose = result
        # 자세(rx,ry,rz)는 현재 TCP 자세 유지(수직 그립 가정)
        response.success = True
        response.depth_position = [
            float(base_xyz[0]), float(base_xyz[1]), float(base_xyz[2]),
            float(cur_pose[3]), float(cur_pose[4]), float(cur_pose[5]),
        ]
        response.confidence = float(best["conf"])
        response.message = f"{target} found"
        self.get_logger().info(
            f"{target} -> base {response.depth_position[:3]}"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    DR_init.__dsr__node = node      # §1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.pipeline is not None:
            try:
                node.pipeline.stop()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
