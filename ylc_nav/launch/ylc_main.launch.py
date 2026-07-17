# /userdata/dev_ws/src/origincar/ylc_nav/launch/ylc_main.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    # 1. 定义主节点 (你的业务逻辑)
    # ylc_main_node = Node(
    #     package='ylc_nav',
    #     executable='ylc_main.py',
    #     name='ylc_main_node',
    #     output='screen',
    #     emulate_tty=True
    # )
    start_button_node = Node(
        package='ylc_nav',
        executable='start_button_node.py',
        name='start_button_node',
        output='screen',
        emulate_tty=True
    )
    # 2. 包含 go_pose 启动文件 (负责导航)
    # 注意：这里不需要传参了，参数直接写在 go_pose.launch.py 里
    go_pose_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ylc_nav'),
                'launch',
                'go_pose.launch.py'
            ])
        )
    )

    # 3. 返回包含两个部分的启动描述
    return LaunchDescription([
        #ylc_main_node,
        start_button_node,
        go_pose_launch
    ])