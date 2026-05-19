"""
wearing_check.py
CheckWearing 서비스 서버 (/check_wearing).

손목 카메라로 사용자를 비춰 얼굴(face) 박스 기준으로 지정 소품들이
착용되었는지 Bounding Box IoU 로 판별. 다중프레임 집계로 안정화.
이 노드는 DSR API 를 사용하지 않으므로 DR_init 연결 생략.
"""

import numpy as np
import rclpy
from rclpy.node import Node

from doocut_interfaces.srv import CheckWearing
from object_detection.yolo import YoloDetector, iou_xyxy

NODE_NAME = "wearing_check_node"

DEFAULT_IOU_TH = 0.10        # 얼굴 대비 소품은 IoU 가 작아도 착용으로 간주
MULTI_FRAMES = 7             # 다중프레임 집계 횟수
NEED_HIT_RATIO = 0.4         # 프레임 중 이 비율 이상 검출 시 착용 확정


class WearingCheckNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.detector = YoloDetector(conf=0.4)

        self.pipeline = None
        self._init_realsense()

        self.srv = self.create_service(
            CheckWearing, "/check_wearing", self.handle_request
        )
        self.get_logger().info("CheckWearing 서버 준비 (/check_wearing)")

    def _init_realsense(self):
        try:
            import pyrealsense2 as rs
            self.pipeline = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.pipeline.start(cfg)
            self.get_logger().info("RealSense color 시작")
        except Exception as e:
            self.get_logger().warn(f"RealSense 비활성 (모의 모드): {e}")
            self.pipeline = None

    def _grab(self):
        if self.pipeline is None:
            return None
        frames = self.pipeline.wait_for_frames()
        c = frames.get_color_frame()
        if not c:
            return None
        return np.asanyarray(c.get_data())

    def handle_request(self, request, response):
        required = list(request.required_props)
        iou_th = request.iou_threshold if request.iou_threshold > 0 \
            else DEFAULT_IOU_TH

        if not required:
            response.all_worn = False
            response.message = "required_props empty"
            return response

        hit_count = {p: 0 for p in required}
        best_iou = {p: 0.0 for p in required}
        face_seen = 0

        for _ in range(MULTI_FRAMES):
            frame = self._grab()
            if frame is None:
                continue
            dets = self.detector.detect(frame)
            faces = [d for d in dets if d["name"] == "face"]
            if not faces:
                continue
            face_seen += 1
            face_box = max(faces, key=lambda d: d["conf"])["box"]

            for prop in required:
                pd = [d for d in dets if d["name"] == prop]
                if not pd:
                    continue
                iou = max(iou_xyxy(face_box, d["box"]) for d in pd)
                if iou >= iou_th:
                    hit_count[prop] += 1
                    best_iou[prop] = max(best_iou[prop], iou)

        worn, missing, scores = [], [], []
        denom = max(1, face_seen)
        for prop in required:
            if hit_count[prop] / denom >= NEED_HIT_RATIO:
                worn.append(prop)
                scores.append(float(best_iou[prop]))
            else:
                missing.append(prop)

        response.all_worn = (len(missing) == 0 and face_seen > 0)
        response.worn_props = worn
        response.missing_props = missing
        response.iou_scores = scores
        if face_seen == 0:
            response.message = "face not detected"
        elif response.all_worn:
            response.message = "all props worn"
        else:
            response.message = f"missing: {missing}"
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = WearingCheckNode()
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
