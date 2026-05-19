"""
system.launch.py
인생DOO컷 최종 데모 런치 — 전체 노드 일괄 기동.

순서 의도:
  1) doocut_interfaces 는 빌드 의존(런타임 노드 없음)
  2) 비전/음성/TTS/로봇 서버 노드 먼저
  3) photo_motion 액션 서버
  4) manager_node (마스터) — 약간 지연 후 기동해 서버들 준비 보장

주의(두산 규칙): 여러 노드가 DSR 드라이버에 동시 접근하므로 각 노드
내부에서 random.sleep(0.1~1.0) 처리됨. 여기서는 manager 만 추가 지연.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    SetEnvironmentVariable
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    set_pythonpath = SetEnvironmentVariable(
    name='PYTHONPATH',
    value=os.environ.get('PYTHONPATH', '') +
          ':/home/yoon/doocut_ws/src/doosan-robot2/dsr_common2/imp'
)
    use_robot = LaunchConfiguration("use_robot")

    declare_use_robot = DeclareLaunchArgument(
        "use_robot", default_value="true",
        description="false 면 로봇/촬영 노드 제외 (비전·음성만)",
    )

    voice_node = Node(
        package="voice_processing",
        executable="get_keyword",
        name="get_keyword_node",
        output="screen",
    )

    detection_node = Node(
        package="object_detection",
        executable="detection",
        name="detection_node",
        namespace="dsr01",
        output="screen",
    )

    wearing_node = Node(
        package="object_detection",
        executable="wearing_check",
        name="wearing_check_node",
        output="screen",
    )

    tts_node = Node(
        package="doocut_tts",
        executable="tts_node",
        name="tts_node",
        output="screen",
    )

    robot_node = Node(
        package="robot_control",
        executable="robot_control",
        name="robot_control_node",
        namespace="dsr01",
        output="screen",
        condition=None,
    )

    photo_motion_node = Node(
        package="robot_control",
        executable="photo_motion",
        name="photo_motion_server",
        namespace="dsr01",
        output="screen",
    )

    # manager 는 서버들이 뜬 뒤 기동되도록 5초 지연
    manager_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="photo_booth_manager",
                executable="manager_node",
                name="photo_booth_manager",
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        set_pythonpath,
        declare_use_robot,
        voice_node,
        detection_node,
        wearing_node,
        tts_node,
        robot_node,
        photo_motion_node,
        manager_node,
    ])
