# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""
    RMV1 = Node(
        package="vision",
        executable="rmvision",
        # ns="RM1",
        # remappings=[
        #     ('/motion_capture', '/RM1'),
        # ]
    )
    RMV2 = Node(
        package="vision",
        executable="rmvision",
        name="RMV2",
        # remappings=[
        #     ('/motion_capture', '/RM2'),
        # ]
    )
    RMV3 = Node(
        package="vision",
        executable="rmvision",
        name="RMV3",
        # remappings=[
        #     ('/motion_capture', '/RM3'),
        # ]
    )
    RMV4 = Node(
        package="vision",
        executable="rmvision",
        name="RMV4",
        # remappings=[
        #     ('/motion_capture', '/RM4'),
        # ]
    )
    RMV5 = Node(
        package="vision",
        executable="rmvision",
        name="RMV5",
        # remappings=[
        #     ('/motion_capture', '/RM5'),
        # ]
    )
    RMV6 = Node(
        package="vision",
        executable="rmvision",
        name="RMV6",
        # remappings=[
        #     ('/motion_capture', '/RM6'),
        # ]
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件 RM4, RM5, RM6
    launch_description = LaunchDescription(
        [RMV1, ])
    # 返回让ROS2根据launch描述执行节点
    return launch_description