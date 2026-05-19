import rclpy
import cv2
import json
import os
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
from object_detection.realsense import ImgNode

PACKAGE_PATH = get_package_share_directory("object_detection")
YOLO_MODEL_PATH = os.path.join(PACKAGE_PATH, "resource", "best.pt")
YOLO_JSON_PATH = os.path.join(PACKAGE_PATH, "resource", "class_name_tool.json")


def main():
    rclpy.init()
    node = ImgNode()

    model = YOLO(YOLO_MODEL_PATH)

    # 모델 내장 클래스 목록 출력 → 그립 클래스명 확인용
    print("=" * 50)
    print("[visualize] Model class map:")
    for idx, name in model.names.items():
        print(f"  {idx}: {name}")
    print("=" * 50)
    print("Press 'q' to quit.")

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        frame = node.get_color_frame()
        if frame is None:
            continue

        results = model(frame, verbose=False)
        
        # 키포인트 정보가 있는지 확인
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

            # 모델이 여러 개의 키포인트(빨, 노, 흰 등)를 반환한다면,
            # 인덱스별로 색깔을 다르게 해서 그립니다.
            if has_keypoints:
                kpts = results[0].keypoints.xy[i].tolist()  # [[x, y], [x, y], ...]
                # 0번: 빨강, 1번: 노랑, 2번: 하양 (BGR)
                colors = [(0, 0, 255), (0, 255, 255), (255, 255, 255)]
                for idx, (kx, ky) in enumerate(kpts):
                    # 0,0으로 반환되는 경우는 인식 실패한 점이므로 제외
                    if kx > 0 and ky > 0:
                        c = colors[idx] if idx < len(colors) else (0, 0, 0)
                        cv2.circle(frame, (int(kx), int(ky)), 6, c, -1)
                        cv2.circle(frame, (int(kx), int(ky)), 10, (0, 0, 0), 1)
                        # 점 옆에 인덱스 번호 글씨 표시 (0, 1, 2)
                        cv2.putText(frame, f"KP:{idx}", (int(kx)+10, int(ky)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2)

        cv2.imshow("YOLO Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()
