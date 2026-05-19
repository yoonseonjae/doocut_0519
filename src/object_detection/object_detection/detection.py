import json
import numpy as np
import rclpy
from rclpy.node import Node
from typing import Any, Callable, Optional, Tuple
import cv2
from cv_bridge import CvBridge

from ament_index_python.packages import get_package_share_directory
from od_msg.srv import SrvDepthPosition
from sensor_msgs.msg import Image
from object_detection.realsense import ImgNode
from object_detection.yolo import YoloModel, YOLO_JSON_PATH


PACKAGE_NAME = 'object_detection'
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)


class ObjectDetectionNode(Node):
    def __init__(self, model_name = 'yolo'):
        super().__init__('object_detection_node')
        self.img_node = ImgNode()
        self.model = self._load_model(model_name)
        self.intrinsics = self._wait_for_valid_data(
            self.img_node.get_camera_intrinsic, "camera intrinsics"
        )
        self.create_service(
            SrvDepthPosition,
            'get_3d_position',
            self.handle_get_depth
        )

        self.bridge = CvBridge()
        self.detection_pub = self.create_publisher(Image, '/detection_image', 10)
        with open(YOLO_JSON_PATH, 'r', encoding='utf-8') as f:
            self.class_dict = json.load(f)
        self.create_timer(0.1, self._publish_detection_image)

        self.get_logger().info("ObjectDetectionNode initialized.")

    def _publish_detection_image(self):
        """주기적으로 현재 바운딩박스를 그린 이미지를 publish."""
        rclpy.spin_once(self.img_node, timeout_sec=0.0)
        frame = self.img_node.get_color_frame()
        if frame is None:
            return
        frame = frame.copy()
        results = self.model.model([frame], verbose=False)
        has_keypoints = results[0].keypoints is not None
        
        for i, (box, score, label) in enumerate(zip(
            results[0].boxes.xyxy.tolist(),
            results[0].boxes.conf.tolist(),
            results[0].boxes.cls.tolist(),
        )):
            if score < 0.5:
                continue
            x1, y1, x2, y2 = map(int, box)
            name = results[0].names[int(label)]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{name} {score:.2f}", (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        
            if has_keypoints:
                kpts = results[0].keypoints.xy[i].tolist()
                for kx, ky in kpts:
                    if kx > 0 and ky > 0:
                        cv2.circle(frame, (int(kx), int(ky)), 6, (0, 0, 255), -1)
                        cv2.circle(frame, (int(kx), int(ky)), 10, (255, 255, 255), 1)
                        
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.detection_pub.publish(msg)

    def _load_model(self, name):
        """모델 이름에 따라 인스턴스를 반환합니다."""
        if name.lower() == 'yolo':
            return YoloModel()
        raise ValueError(f"Unsupported model: {name}")

    def handle_get_depth(self, request, response):
        """클라이언트 요청을 처리해 3D 좌표를 반환합니다."""
        self.get_logger().info(f"Received request: {request}")
        coords = self._compute_position(request.target)
        response.depth_position = [float(x) for x in coords]
        return response

    def _compute_position(self, target):
        """이미지를 처리해 객체의 카메라 좌표를 계산합니다."""
        rclpy.spin_once(self.img_node)

        box, score, keypoint = self.model.get_best_detection(self.img_node, target)
        if box is None or score is None:
            self.get_logger().warn("No detection found.")
            return 0.0, 0.0, 0.0
        
        self.get_logger().info(f"Detection: box={box}, score={score}, keypoint={keypoint}")
        
        if keypoint is not None and keypoint[0] > 0 and keypoint[1] > 0:
            cx, cy = int(keypoint[0]), int(keypoint[1])
            self.get_logger().info(f"Targeting Keypoint center: ({cx}, {cy})")
        else:
            cx, cy = map(int, [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2])
            self.get_logger().info(f"Targeting Bbox center: ({cx}, {cy})")
        cz = self._get_depth(cx, cy)
        if cz is None:
            self.get_logger().warn("Depth out of range.")
            return 0.0, 0.0, 0.0

        return self._pixel_to_camera_coords(cx, cy, cz)

    def _get_depth(self, x, y):
        """픽셀 좌표의 depth 값을 안전하게 읽어옵니다."""
        frame = self._wait_for_valid_data(self.img_node.get_depth_frame, "depth frame")
        try:
            return frame[y, x]
        except IndexError:
            self.get_logger().warn(f"Coordinates ({x},{y}) out of range.")
            return None

    def _wait_for_valid_data(self, getter, description):
        """getter 함수가 유효한 데이터를 반환할 때까지 spin 하며 재시도합니다."""
        data = getter()
        while data is None or (isinstance(data, np.ndarray) and not data.any()):
            rclpy.spin_once(self.img_node)
            self.get_logger().info(f"Retry getting {description}.")
            data = getter()
        return data

    def _pixel_to_camera_coords(self, x, y, z):
        """픽셀 좌표와 intrinsics를 이용해 카메라 좌표계로 변환합니다."""
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        ppx = self.intrinsics['ppx']
        ppy = self.intrinsics['ppy']
        return (
            (x - ppx) * z / fx,
            (y - ppy) * z / fy,
            z
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
