from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ROBOTS = ['RM1', 'RM2', 'RM3']


def _detector(robot_name: str) -> Node:
    publish_debug = LaunchConfiguration('publish_debug')
    return Node(
        package='rm_grip_vision',
        executable='grip_vision_detector',
        name=f'{robot_name}_grip_vision',
        output='screen',
        parameters=[{
            'robot_name': robot_name,
            'image_topic': f'/{robot_name}/camera/image_color',
            'caught_topic': f'/{robot_name}/grip_vision/caught',
            'confidence_topic': f'/{robot_name}/grip_vision/confidence',
            'debug_image_topic': f'/{robot_name}/grip_vision/debug_image',
            'publish_debug': publish_debug,
        }],
    )


def generate_launch_description():
    visual_wait_sec = LaunchConfiguration('visual_wait_sec')
    visual_timeout_sec = LaunchConfiguration('visual_timeout_sec')
    publish_debug = DeclareLaunchArgument(
        'publish_debug',
        default_value='false',
        description='Publish debug images with ROI/contours on /RMx/grip_vision/debug_image',
    )
    return LaunchDescription([
        publish_debug,
        DeclareLaunchArgument(
            'visual_wait_sec',
            default_value='0.8',
            description='How long strategy waits for a fresh visual decision after gripper closes',
        ),
        DeclareLaunchArgument(
            'visual_timeout_sec',
            default_value='0.7',
            description='Max age of /RMx/grip_vision/caught used by strategy',
        ),
        *[_detector(robot) for robot in ROBOTS],
        Node(
            package='rmcontrol',
            executable='strategy',
            name='strategy',
            output='screen',
            parameters=[{
                'use_visual_catch': True,
                'visual_wait_sec': visual_wait_sec,
                'visual_timeout_sec': visual_timeout_sec,
            }],
        ),
    ])
