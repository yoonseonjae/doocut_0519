"""
vision_only.launch.py
비전 노드만 기동 — 로봇 미연결 환경에서 YOLO/좌표/착용검사 단위 테스트.
detection_node 는 get_current_posx 호출 시 DSR 없으면 단위행렬 폴백.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="object_detection",
            executable="detection",
            name="detection_node",
            namespace="dsr01",
            output="screen",
        ),
        Node(
            package="object_detection",
            executable="wearing_check",
            name="wearing_check_node",
            output="screen",
        ),
    ])
