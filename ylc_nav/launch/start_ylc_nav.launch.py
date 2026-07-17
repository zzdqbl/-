import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    # 包路径
    wheeltec_robot_dir = get_package_share_directory('origincar_base')
    wheeltec_launch_dir = os.path.join(wheeltec_robot_dir, 'launch')
    
    ylc_nav_dir = get_package_share_directory('ylc_nav')    
    wheeltec_nav_dir = get_package_share_directory('wheeltec_nav2')
    wheeltec_nav_launch_dir = os.path.join(wheeltec_nav_dir, 'launch')

    # 地图路径
    map_dir = os.path.join(ylc_nav_dir, 'maps')
    map_file = LaunchConfiguration('map', default='/userdata/dev_ws/src/origincar/ylc_nav/maps/map1.yaml')

    # ==============================================
    # 模式切换：True=SLAM建图   False=AMCL定位
    # ==============================================
    slam_mode = LaunchConfiguration('slam_mode', default='False')

    # 声明启动参数
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=map_file,
        description='地图路径'
    )

    declare_slam_mode_arg = DeclareLaunchArgument(
        'slam_mode',
        default_value='False',
        description='True=SLAM建图模式，False=AMCL定位导航模式'
    )

    # 循环导航节点
    waypoint_node = Node(
        name='waypoint_cycle',
        package='nav2_waypoint_cycle',
        executable='nav2_waypoint_cycle',
    )

    # 启动机器人底盘
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wheeltec_launch_dir, 'origincar_bringup.launch.py')
        ),
    )

    # ==============================================
    # 模式 1：SLAM 建图 → 使用 ylc_nav2.yaml
    # ==============================================
    nav_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wheeltec_nav_launch_dir, 'bringup_launch.py')
        ),
        condition=IfCondition(slam_mode),
        launch_arguments={
            'map': map_file,
            'slam': 'True',
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(ylc_nav_dir, 'config', 'ylc_nav2.yaml')
        }.items(),
    )
    # ==============================================
    # 模式 2：AMCL 定位 → 使用 ylc_nav2_amcl.yaml
    # ==============================================
    nav_amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wheeltec_nav_launch_dir, 'bringup_launch.py')
        ),
        condition=UnlessCondition(slam_mode),  # <--- 修复关键！
        launch_arguments={
            'map': map_file,
            'slam': 'False',
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(ylc_nav_dir, 'config', 'ylc_nav2_teb_amcl.yaml')
        }.items(),
    )

    initial_pose_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='initial_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom_combined'],
        parameters=[{'use_sim_time': False}],
        condition=UnlessCondition(slam_mode)  # <--- 只在 AMCL 模式运行
    )
    # 组装
    ld = LaunchDescription()
    ld.add_action(declare_map_arg)
    ld.add_action(declare_slam_mode_arg)
    ld.add_action(waypoint_node)
    ld.add_action(robot_bringup)
    ld.add_action(nav_slam)
    ld.add_action(nav_amcl)
    ld.add_action(initial_pose_pub)
    return ld
