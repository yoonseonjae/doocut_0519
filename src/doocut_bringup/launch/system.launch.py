from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    system.launch.py
    전체 시스템을 한 번에 실행합니다.

    실행 순서:
      1. get_keyword_node   (voice_processing)  → /get_keyword 서비스 준비
      2. object_detection_node (object_detection) → /get_3d_position 서비스 준비
      3. robot_control_node (robot_control)       → 두 서비스에 순서대로 요청

    사용법:
      ros2 launch doocut_bringup system.launch.py
    """
    bringup_dir = get_package_share_directory('doocut_bringup')

    voice_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'voice.launch.py')
        )
    )

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'vision.launch.py')
        )
    )

    robot_node = Node(
        package='robot_control',
        executable='robot_control',
        name='robot_control_node',
        output='screen',
    )

    return LaunchDescription([
        voice_launch,
        vision_launch,
        robot_node,
    ])
