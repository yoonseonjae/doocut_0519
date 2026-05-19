"""
debug.launch.py
파이프라인 흐름 디버그용 — 음성 + TTS + manager 만 기동.
비전/로봇 노드는 제외하므로 manager 가 각 서비스 timeout 후
폴백 경로(스킵/모의 캡처)로 진행하는지 확인할 때 사용.
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="doocut_tts",
            executable="tts_node",
            name="tts_node",
            output="screen",
        ),
        Node(
            package="voice_processing",
            executable="get_keyword",
            name="get_keyword_node",
            output="screen",
        ),
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package="photo_booth_manager",
                    executable="manager_node",
                    name="photo_booth_manager",
                    output="screen",
                )
            ],
        ),
    ])
