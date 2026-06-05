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
    use_visual_catch = LaunchConfiguration('use_visual_catch')
    visual_wait_sec = LaunchConfiguration('visual_wait_sec')
    visual_timeout_sec = LaunchConfiguration('visual_timeout_sec')
    use_los_catch_approach = LaunchConfiguration('use_los_catch_approach')
    los_align_distance = LaunchConfiguration('los_align_distance')
    los_align_threshold = LaunchConfiguration('los_align_threshold')
    los_slow_distance = LaunchConfiguration('los_slow_distance')
    los_min_step = LaunchConfiguration('los_min_step')
    los_max_step = LaunchConfiguration('los_max_step')
    los_final_distance = LaunchConfiguration('los_final_distance')
    los_max_forward_speed = LaunchConfiguration('los_max_forward_speed')
    publish_debug = DeclareLaunchArgument(
        'publish_debug',
        default_value='false',
        description='Publish debug images with ROI/contours on /RMx/grip_vision/debug_image',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=_default_config_file(),
            description='YAML file with shared rm_grip_vision detector/tuner parameters',
        ),
        publish_debug,
        DeclareLaunchArgument(
            'use_visual_catch',
            default_value='true',
            description='Use /RM1..3/grip_vision/caught after gripper closes',
        ),
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
        DeclareLaunchArgument(
            'use_los_catch_approach',
            default_value='true',
            description='Use LOS-style align-then-slow-approach logic for target_code=30',
        ),
        DeclareLaunchArgument(
            'los_align_distance',
            default_value='0.75',
            description='Distance to ball where catcher starts prioritising heading alignment',
        ),
        DeclareLaunchArgument(
            'los_align_threshold',
            default_value='0.16',
            description='Allowed LOS heading error in radians before slow approach',
        ),
        DeclareLaunchArgument(
            'los_slow_distance',
            default_value='0.55',
            description='Distance to ball where approach step/speed begins tapering down',
        ),
        DeclareLaunchArgument(
            'los_min_step',
            default_value='0.04',
            description='Minimum intermediate target step during slow approach',
        ),
        DeclareLaunchArgument(
            'los_max_step',
            default_value='0.22',
            description='Maximum intermediate target step during slow approach',
        ),
        DeclareLaunchArgument(
            'los_final_distance',
            default_value='0.30',
            description='Desired final robot-ball distance before check_catchable closes the gripper',
        ),
        DeclareLaunchArgument(
            'los_max_forward_speed',
            default_value='0.35',
            description='Max relative forward command during LOS catch approach',
        ),
        *[_detector(robot) for robot in ROBOTS],
        Node(
            package='rmcontrol',
            executable='strategy',
            name='strategy',
            output='screen',
            parameters=[{
                'use_visual_catch': ParameterValue(use_visual_catch, value_type=bool),
                'visual_wait_sec': ParameterValue(visual_wait_sec, value_type=float),
                'visual_timeout_sec': ParameterValue(visual_timeout_sec, value_type=float),
                'use_los_catch_approach': ParameterValue(
                    use_los_catch_approach,
                    value_type=bool,
                ),
                'los_align_distance': ParameterValue(los_align_distance, value_type=float),
                'los_align_threshold': ParameterValue(los_align_threshold, value_type=float),
                'los_slow_distance': ParameterValue(los_slow_distance, value_type=float),
                'los_min_step': ParameterValue(los_min_step, value_type=float),
                'los_max_step': ParameterValue(los_max_step, value_type=float),
                'los_final_distance': ParameterValue(los_final_distance, value_type=float),
                'los_max_forward_speed': ParameterValue(los_max_forward_speed, value_type=float),
            }],
        ),
    ])
