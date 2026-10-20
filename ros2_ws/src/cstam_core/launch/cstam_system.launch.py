import os
from typing import Any, List
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def launch_setup(context: LaunchContext, *args, **kwargs) -> List[Any]:
    pkg_cstam_gazebo = get_package_share_directory('cstam_gazebo')
    pkg_cstam_navigation = get_package_share_directory('cstam_navigation')

    world_str = context.launch_configurations.get('world', 'restaurant.world')
    is_resto_arbi = 'resto_arbi' in world_str

    # World-specific presets
    if is_resto_arbi:
        default_map = os.path.join(pkg_cstam_navigation, 'maps', 'resto_arbi_map.yaml')
        default_wps = os.path.join(pkg_cstam_navigation, 'config', 'resto_arbi_waypoints.yaml')
        default_params = os.path.join(pkg_cstam_navigation, 'config', 'nav2_params_resto_arbi.yaml')
        default_sx = '-10.0'
        default_sy = '-1.45'
        default_sz = '0.1'
        default_syaw = '0.0'
    else:
        default_map = os.path.join(pkg_cstam_navigation, 'maps', 'cstam_map.yaml')
        default_wps = os.path.join(pkg_cstam_navigation, 'config', 'waypoints.yaml')
        default_params = os.path.join(pkg_cstam_navigation, 'config', 'nav2_params.yaml')
        default_sx = '7.06'
        default_sy = '-12.0'
        default_sz = '0.1'
        default_syaw = '1.57'

    # Respect user overrides or use smart world presets
    user_map = context.launch_configurations.get('map', '')
    active_map = user_map if user_map else default_map

    user_params = context.launch_configurations.get('params_file', '')
    active_params = user_params if user_params else default_params

    user_wps = context.launch_configurations.get('waypoints_file', '')
    active_wps = user_wps if user_wps else default_wps

    user_sx = context.launch_configurations.get('spawn_x', '')
    active_sx = user_sx if user_sx and user_sx != 'AUTO' else default_sx

    user_sy = context.launch_configurations.get('spawn_y', '')
    active_sy = user_sy if user_sy and user_sy != 'AUTO' else default_sy

    user_sz = context.launch_configurations.get('spawn_z', '')
    active_sz = user_sz if user_sz and user_sz != 'AUTO' else default_sz

    user_syaw = context.launch_configurations.get('spawn_yaw', '')
    active_syaw = user_syaw if user_syaw and user_syaw != 'AUTO' else default_syaw

    use_sim_time = context.launch_configurations.get('use_sim_time', 'true')
    launch_gazebo = context.launch_configurations.get('launch_gazebo', 'true').lower() == 'true'
    launch_nav = context.launch_configurations.get('launch_nav', 'true').lower() == 'true'
    launch_rviz = context.launch_configurations.get('launch_rviz', 'true').lower() == 'true'

    # Battery policy shared by the task manager and docking controller
    battery_policy = {
        'low_battery_threshold': 25.0,
        'critical_battery_threshold': 15.0,
        'min_recharge_pct': 80.0,
    }

    nodes: List[Any] = []

    # 1. Gazebo + Robot Spawn Launch
    if launch_gazebo:
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_cstam_gazebo, 'launch', 'spawn_cstam_robot.launch.py')
            ),
            launch_arguments={
                'world': world_str,
                'spawn_x': active_sx,
                'spawn_y': active_sy,
                'spawn_z': active_sz,
                'spawn_yaw': active_syaw,
                'headless': context.launch_configurations.get('headless', 'false'),
            }.items()
        )
        nodes.append(gazebo_launch)

    # 2. Navigation Launch (Nav2 + Map Server + AMCL)
    if launch_nav:
        nav_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_cstam_navigation, 'launch', 'navigation.launch.py')
            ),
            launch_arguments={
                'map': active_map,
                'params_file': active_params,
            }.items()
        )
        nodes.append(nav_launch)

    # 3. RViz2 3D Navigation Visualizer
    if launch_rviz:
        rviz_config_file = os.path.join(pkg_cstam_navigation, 'config', 'cstam_nav2.rviz')
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time.lower() == 'true'}],
            output='screen'
        )
        nodes.append(rviz_node)

    # 4. Delivery Task Manager Node
    task_manager_node = Node(
        package='cstam_core',
        executable='delivery_task_manager',
        name='delivery_task_manager',
        parameters=[{
            'use_sim_time': use_sim_time.lower() == 'true',
            'waypoints_file': active_wps,
            **battery_policy,
        }],
        output='screen'
    )
    nodes.append(task_manager_node)

    # 5. Docking Controller Node
    docking_controller_node = Node(
        package='cstam_core',
        executable='docking_controller',
        name='docking_controller',
        parameters=[{
            'use_sim_time': use_sim_time.lower() == 'true',
            'idle_timeout': 10.0,
            **battery_policy,
        }],
        output='screen'
    )
    nodes.append(docking_controller_node)

    # 6. Physical 24V Li-ion Battery & State Simulator Node
    battery_simulator_node = Node(
        package='cstam_core',
        executable='battery_simulator',
        name='battery_simulator',
        parameters=[{
            'use_sim_time': use_sim_time.lower() == 'true',
            'initial_battery': 100.0,
            'time_scale': 15.0
        }],
        output='screen'
    )
    nodes.append(battery_simulator_node)

    # 7. Dynamic Pedestrian Obstacle Controller Node
    dynamic_obs_node = Node(
        package='cstam_core',
        executable='dynamic_obstacle_controller',
        name='dynamic_obstacle_controller',
        parameters=[{
            'use_sim_time': use_sim_time.lower() == 'true',
            'auto_patrol': True,
            'speed': 0.40,
            'start_x': -2.0 if is_resto_arbi else -1.5,
            'start_y': 1.0 if is_resto_arbi else -1.3,
            'start_yaw': 1.57,  # matches the dynamic_obstacle <include> pose in both worlds
            'patrol_min': -1.0 if is_resto_arbi else -3.2,
            'patrol_max': 3.0 if is_resto_arbi else 0.5,
        }],
        output='screen'
    )
    nodes.append(dynamic_obs_node)

    return nodes


def generate_launch_description():
    # Detect if nav2_bringup is installed
    try:
        get_package_share_directory('nav2_bringup')
        nav2_available = True
    except PackageNotFoundError:
        nav2_available = False

    default_nav = 'true' if nav2_available else 'false'

    # Detect if Gazebo & xacro dependencies are installed
    try:
        import xacro  # noqa: F401  (availability probe)
        get_package_share_directory('ros_gz_sim')
        get_package_share_directory('ros_gz_bridge')
        gazebo_available = True
    except (ImportError, PackageNotFoundError):
        gazebo_available = False

    default_gazebo = 'true' if gazebo_available else 'false'
    default_sim_time = 'true' if gazebo_available else 'false'

    # Declare Launch Arguments
    args = [
        DeclareLaunchArgument('world', default_value='restaurant.world',
                              description='World file name (e.g. resto_arbi.world or restaurant.world)'),
        DeclareLaunchArgument('map', default_value='',
                              description='Path to map yaml (leave empty for automatic world mapping)'),
        DeclareLaunchArgument('params_file', default_value='',
                              description='Path to nav2 params (leave empty for automatic world params)'),
        DeclareLaunchArgument('waypoints_file', default_value='',
                              description='Path to waypoints yaml (leave empty for automatic world waypoints)'),
        DeclareLaunchArgument('launch_gazebo', default_value=default_gazebo,
                              description='Launch Gazebo Sim simulation'),
        DeclareLaunchArgument('launch_nav', default_value=default_nav,
                              description='Launch Nav2 navigation stack'),
        DeclareLaunchArgument('use_sim_time', default_value=default_sim_time,
                              description='Use simulation clock'),
        DeclareLaunchArgument('spawn_x', default_value='AUTO',
                              description='X coordinate for spawn (AUTO = world dock)'),
        DeclareLaunchArgument('spawn_y', default_value='AUTO',
                              description='Y coordinate for spawn (AUTO = world dock)'),
        DeclareLaunchArgument('spawn_z', default_value='AUTO',
                              description='Z coordinate for spawn'),
        DeclareLaunchArgument('spawn_yaw', default_value='AUTO',
                              description='Yaw rotation for spawn (AUTO = facing aisle)'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run Gazebo without GUI (server + headless rendering)'),
        DeclareLaunchArgument('launch_rviz', default_value='true',
                              description='Launch RViz2 with 3D Navigation visualization'),
    ]

    launch_items: List[Any] = list(args)

    if not gazebo_available:
        launch_items.append(LogInfo(
            msg="[CSTAM NOTICE] Gazebo simulation dependencies are not installed; skipping Gazebo bringup."
        ))

    if not nav2_available:
        launch_items.append(LogInfo(
            msg="[CSTAM NOTICE] 'nav2_bringup' is not installed; skipping Nav2 bringup."
        ))

    launch_items.append(OpaqueFunction(function=launch_setup))

    return LaunchDescription(launch_items)
