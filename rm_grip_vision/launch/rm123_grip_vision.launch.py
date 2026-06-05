import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ROBOTS = ['RM1', 'RM2', 'RM3']


def _default_config_file() -> str:
    return os.path.join(
        get_package_share_directory('rm_grip_vision'),
        'config',
        'grip_vision_default.yaml',
    )


def _detector(robot_name: str) -> Node:
    config_file = LaunchConfiguration('config_file')
    publish_debug = LaunchConfiguration('publish_debug')
    publish_debug_value = ParameterValue(publish_debug, value_type=bool)
    return Node(
        package='rm_grip_vision',
        executable='grip_vision_detector',
        name=f'{robot_name}_grip_vision',
        output='screen',
        parameters=[
            config_file,
            {
                'robot_name': robot_name,
                'image_topic': f'/{robot_name}/camera/image_color',
                'caught_topic': f'/{robot_name}/grip_vision/caught',
                'confidence_topic': f'/{robot_name}/grip_vision/confidence',
                'caught_score_topic': f'/{robot_name}/grip_vision/caught_score',
                'debug_image_topic': f'/{robot_name}/grip_vision/debug_image',
                'publish_debug': publish_debug_value,
            },
        ],
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=_default_config_file(),
            description='YAML file with shared rm_grip_vision detector/tuner parameters',
        ),
        DeclareLaunchArgument(
            'publish_debug',
            default_value='false',
            description='Publish debug images with ROI/contours on /RMx/grip_vision/debug_image',
        ),
        *[_detector(robot) for robot in ROBOTS],
    ])
