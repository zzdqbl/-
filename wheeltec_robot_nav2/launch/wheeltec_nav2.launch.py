import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    wheeltec_robot_dir = get_package_share_directory('origincar_base')
    wheeltec_launch_dir = os.path.join(wheeltec_robot_dir, 'launch')
    ylc_nav_dir = get_package_share_directory('ylc_nav')    
    wheeltec_nav_dir = get_package_share_directory('wheeltec_nav2')
    wheeltec_nav_launchr = os.path.join(wheeltec_nav_dir, 'launch')


    map_dir = os.path.join(ylc_nav_dir, 'maps')
    map_file = LaunchConfiguration('map', default=os.path.join(
        map_dir, 'test.yaml'))

    #Modify the model parameter file, the options are:
    #param_mini_akm.yaml/param_mini_4wd.yaml/param_mini_diff.yaml/
    #param_mini_mec.yaml/param_mini_omni.yaml/param_mini_tank.yaml/
    #param_senior_akm.yaml/param_senior_diff.yaml/param_senior_mec_bs.yaml
    #param_senior_mec_dl.yaml/param_top_4wd_bs.yaml/param_top_4wd_dl.yaml
    #param_top_akm_dl.yaml/param_four_wheel_diff_dl.yaml/param_four_wheel_diff_bs.yaml

    param_dir = os.path.join(wheeltec_nav_dir, 'param','wheeltec_params')
    param_file = LaunchConfiguration('params', default=os.path.join(
        ylc_nav_dir,'config', 'ylc_nav2.yaml'))
    # param_file = LaunchConfiguration('params', default=os.path.join(
        # param_dir, 'ylc_nav.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params',
            default_value=param_file,
            description='Full path to param file to load'),
        Node(
            name='waypoint_cycle',
            package='nav2_waypoint_cycle',
            executable='nav2_waypoint_cycle',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [wheeltec_launch_dir, '/origincar_bringup.launch.py']),
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [wheeltec_launch_dir, '/wheeltec_lidar.launch.py']),
        # ),        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [wheeltec_nav_launchr, '/bringup_launch.py']),
            launch_arguments={
                'map': map_file,
                'slam': 'False',
                'use_sim_time': use_sim_time,
                'params_file': param_file}.items(),
        ),

    ])
