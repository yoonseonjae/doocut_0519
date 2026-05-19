import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from cv_bridge import CvBridge
import cv2


class RealSenseSubscriber(Node):

    def __init__(self):
        super().__init__('realsense_subscriber')

        # qos_profile = QoSProfile(
        #     reliability=ReliabilityPolicy.BEST_EFFORT,
        #     durability=DurabilityPolicy.VOLATILE,
        #     history=HistoryPolicy.KEEP_LAST,
        #     depth=10
        # )

        self.bridge = CvBridge()

        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.rgb_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

    def rgb_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            cv2.imshow("RGB Image", frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"RGB Error: {e}")

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Depth 창�졻�� 챙�뮴벭ぢ걔겷�꽓�� (0~255 챙힋짚챙쩌���씲셌ヂ㎳�)
            depth_normalized = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
            depth_colormap = cv2.applyColorMap(depth_normalized.astype('uint8'), cv2.COLORMAP_JET)

            cv2.imshow("Depth Image", depth_colormap)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Depth Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()