from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    vision.launch.py
    object_detection 노드를 실행합니다.
    - RealSense 카메라 구독
    - YOLO 추론
    - /get_3d_position 서비스 서버
    - /detection_image 토픽 publish
    """
    return LaunchDescription([
        Node(
            package='object_detection',
            executable='object_detection',
            name='object_detection_node',
            output='screen',
        ),
    ])
