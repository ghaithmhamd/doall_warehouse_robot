import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Arduino controller node
    arduino_node = Node(
        package='doall',
        executable='arduino_controller',
        output='screen'
    )

    xacro_file = os.path.join(get_package_share_directory('doall'),'model', 'robot.xacro')
    # Robot description
    robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_state_pub_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config}]
        )

    # RPLidar launch
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_c1_launch.py'
            )
        )
    )

    # Nav2_localisation
    nav2_localization_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'localization_launch.py'
    )
    localization_params_path = os.path.join(
        get_package_share_directory('doall'),
        'config',
        'amcl_localization.yaml'
    )
    # Nav2_navigation
    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )
    navigation_params_path = os.path.join(
        get_package_share_directory('doall'),
        'config',
        'navigation.yaml'
    )

    # Map file
    map_file_path = os.path.join(
        get_package_share_directory('doall'),
        'maps',
        'my_room_map.yaml'
    )

    # Nav2_localisation launch
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_path),
        launch_arguments={
                'params_file': localization_params_path,
                'map': map_file_path,
                'use_sim_time': 'false',
                'autostart': 'true',
        }.items()
    )

    # Nav2_navigation launch
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
                'params_file': navigation_params_path,
                'use_sim_time': 'false',
                'autostart': 'true',
        }.items()
    )


    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(arduino_node)
    launchDescriptionObject.add_action(robot_state_pub_node)
    launchDescriptionObject.add_action(rplidar_launch)
    launchDescriptionObject.add_action(localization_launch)
    launchDescriptionObject.add_action(navigation_launch)
    return launchDescriptionObject