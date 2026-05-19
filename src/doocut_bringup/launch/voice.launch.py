from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    voice.launch.py
    get_keyword 노드만 실행합니다.
    - wake word 감지 (hello rokey)
    - STT 5초 수신
    - scene 키워드 추출 → 소품 목록 반환
    """
    return LaunchDescription([
        Node(
            package='voice_processing',
            executable='get_keyword',
            name='get_keyword_node',
            output='screen',
        ),
    ])
