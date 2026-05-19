import cv2
import rclpy
from rclpy.node import Node
import numpy as np

from object_detection.yolo import YoloDetector

class VisualizeNode(Node):
    def __init__(self):
        super().__init__('visualize_node')
        self.detector = YoloDetector(conf=0.4)
        
        self.pipeline = None
        try:
            import pyrealsense2 as rs
            self.pipeline = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.pipeline.start(cfg)
            self.get_logger().info("RealSense color started for visualization.")
            self.get_logger().info("Press 'q' in the image window to quit.")
        except Exception as e:
            self.get_logger().error(f"Failed to start RealSense: {e}")
            self.get_logger().error("Hint: system.launch.py must NOT be running at the same time, because the USB camera can only be accessed by one process!")

        # 30 fps
        self.timer = self.create_timer(0.033, self.timer_callback)

    def timer_callback(self):
        if self.pipeline is None:
            return
        
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=50)
            c = frames.get_color_frame()
            if not c:
                return
            frame = np.asanyarray(c.get_data())
            
            if self.detector.ready:
                # Ultralytics 내부 plot() 함수를 사용하여 박스와 클래스명, 확률을 렌더링
                results = self.detector.model(frame, conf=self.detector.conf, verbose=False)
                if results:
                    annotated_frame = results[0].plot()
                    cv2.imshow("Detection", annotated_frame)
                else:
                    cv2.imshow("Detection", frame)
            else:
                cv2.imshow("Detection", frame)
                
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("Quitting visualization...")
                rclpy.shutdown()
        except Exception as e:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = VisualizeNode()
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
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
