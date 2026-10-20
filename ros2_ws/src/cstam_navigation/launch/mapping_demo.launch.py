import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_cstam_gazebo = get_package_share_directory('cstam_gazebo')
    pkg_cstam_navigation = get_package_share_directory('cstam_navigation')

    rviz_config_file = os.path.join(pkg_cstam_navigation, 'config', 'slam_demo.rviz')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='restaurant.world',
        description='World file name (e.g. restaurant.world, resto_arbi.world)'
    )
    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x',
        default_value='7.06',
        description='Spawn X coordinate'
    )
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y',
        default_value='-12.0',
        description='Spawn Y coordinate'
    )
    spawn_yaw_arg = DeclareLaunchArgument(
        'spawn_yaw',
        default_value='1.57',
        description='Spawn Yaw (radians)'
    )

    # 1. Launch Gazebo Simulation with Restaurant World & BellaBot Robot
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_cstam_gazebo, 'launch', 'spawn_cstam_robot.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw')
        }.items()
    )

    # 2. Launch SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_cstam_navigation, 'launch', 'slam.launch.py')
        )
    )

    # 3. Launch RViz2 with SLAM Demo configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_yaw_arg,
        gazebo_launch,
        slam_launch,
        rviz_node
    ])
